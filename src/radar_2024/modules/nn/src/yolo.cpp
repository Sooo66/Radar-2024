#include <filesystem>
#include <NvOnnxParser.h>
#include <common.h>
#include "srm/nn/yolo.h"
namespace srm::nn
{
  bool Yolo::Initialize(std::string REF_IN model_file, int num_classes)
  {
    model_file_path_ = std::move(model_file);
    num_classes_ = std::move(num_classes);
    std::filesystem::path path(model_file_path_);
    path.replace_extension("cache");
    model_cache_path_ = path.c_str();
    initLibNvInferPlugins(&logger_, "");
    if (!std::filesystem::exists(model_cache_path_))
    {
      if (!BuildEngineFromONNX())
      {
        DLOG(ERROR) << "Failed to build engine from ONNX.";
        return false;
      }
    }
    BuildEngineFromCache();
    if (engine_ == nullptr)
      LOG(ERROR) << "Build CUDA engine failed.";
    if (engine_->getNbBindings() != 2)
    {
      LOG(ERROR) << "Invalid ONNX file type. The ONNX file must be SISO.";
      return false;
    }
    auto shape = engine_->getBindingDimensions(0);
    batches_ = shape.d[0];
    channels_ = shape.d[1];
    input_h_ = shape.d[2];
    input_w_ = shape.d[3];
    checkRuntime(cudaStreamCreate(&stream_));
    execution_context_ = engine_->createExecutionContext();
    input_numel_ = batches_ * channels_ * input_h_ * input_w_;
    checkRuntime(cudaMallocHost(&input_data_host_, input_numel_ * sizeof(float)));
    checkRuntime(cudaMalloc(&input_data_device_, input_numel_ * sizeof(float)));
    output_numel_ = batches_ *
                    (input_w_ / 8 * input_h_ / 8 + input_w_ / 16 * input_h_ / 16 + input_w_ / 32 * input_h_ / 32) *
                    (4 + num_classes_);
    checkRuntime(cudaMallocHost(&output_data_host_, output_numel_ * sizeof(float)));
    checkRuntime(cudaMalloc(&output_data_device_, output_numel_ * sizeof(float)));
    return true;
  }

  std::vector<Objects> Yolo::Infer(cv::Mat image)
  {
    float ro, dw, dh;
    LetterBox(image, ro, dw, dh);
    image.convertTo(image, CV_32F);
    image /= 255.f;
    cv::Mat image_splits[3];
    cv::split(image, image_splits); 
    std::swap(image_splits[0], image_splits[2]);
    for (auto &&image_split : image_splits)
    {
      memcpy(input_data_host_, image_split.data, input_w_ * input_h_ * sizeof(float));
      input_data_host_ += input_w_ * input_h_;
    }
    input_data_host_ -= input_numel_;

    auto start = std::chrono::system_clock::now();
    checkRuntime(cudaMemcpyAsync(input_data_device_, input_data_host_, input_numel_ * sizeof(float),
                                 cudaMemcpyHostToDevice, stream_));
    float *bindings[] = {input_data_device_, output_data_device_};
    execution_context_->enqueueV2((void **)bindings, stream_, nullptr);
    checkRuntime(cudaMemcpyAsync(output_data_host_, output_data_device_, output_numel_ * sizeof(float),
                                 cudaMemcpyDeviceToHost, stream_));
    checkRuntime(cudaStreamSynchronize(stream_));
    auto end = std::chrono::system_clock::now();
    output_data_ = output_data_host_;
    LOG(INFO) << "Detection time cost: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

    std::vector<Objects> objs;
    GetObjects(objs);
    NMS(objs);
    for (auto &&[x1, y1, x2, y2, prob, cls] : objs)
    {
      x1 -= dw, x2 -= dw, y1 -= dh, y2 -= dh;
      x1 /= ro, x2 /= ro, y1 /= ro, y2 /= ro;
    }
    return objs;
  }

  void Yolo::LetterBox(cv::Mat REF_OUT image, float REF_OUT ro, float REF_OUT dw, float REF_OUT dh)
  {
    cv::Size shape = image.size();
    cv::Size new_shape = {input_w_, input_h_};
    ro = std::min(new_shape.width / (float)shape.width, new_shape.height / (float)shape.height);

    // Compute padding
    cv::Size new_unpad = {(int)round(shape.width * ro), (int)round(shape.height * ro)};
    dw = new_shape.width - new_unpad.width,
    dh = new_shape.height - new_unpad.height; // wh padding

    // divide padding into 2 sides
    dw /= 2.0, dh /= 2.0;

    if (shape != new_unpad)
    { // resize
      cv::resize(image, image, new_unpad, 0, 0, cv::INTER_LINEAR);
    }

    int top = round(dh - 0.1), bottom = round(dh + 0.1);
    int left = round(dw - 0.1), right = round(dw + 0.1);
    cv::copyMakeBorder(image, image, top, bottom, left, right, cv::BORDER_CONSTANT, {114, 114, 114}); // add border
  }

  void Yolo::GetObjects(std::vector<Objects> REF_OUT objs)
  {
    int n = (input_w_ / 8 * input_h_ / 8 + input_w_ / 16 * input_h_ / 16 + input_w_ / 32 * input_h_ / 32);
    int m = (4 + num_classes_);
    // m: 8400, n: 48
    // 48 = x, y, w, h, pro..., k * [xo, yo]
    for (int i = 0; i < n; i++)
    {
      float data[m];
      for (int j = 0; j < m; j++)
      {
        data[j] = output_data_[j * n + i];
      }

      int cls = std::max_element(data + 4, data + 4 + num_classes_) - (data + 4);
      float prob = data[4 + cls];
      if (prob < box_conf_thresh_)
      {
        continue;
      }

      Objects obj;
      obj.cls = cls;
      obj.prob = prob;

      float &x = data[0];
      float &y = data[1];
      float &w = data[2];
      float &h = data[3];

      obj.x1 = x - w / 2;
      obj.y1 = y - h / 2;
      obj.x2 = x + w / 2;
      obj.y2 = y + h / 2;

      // for (int j = 4 + num_classes_; j < m; j += 2) {
      //   obj.pts.emplace_back(data[j], data[j + 1]);
      // }

      objs.push_back(obj);
    }
  }

  void Yolo::NMS(std::vector<Objects> REF_OUT objs)
  {
    std::sort(objs.begin(), objs.end(), [](Objects &a, Objects &b)
              { return a.prob > b.prob; });
    if (objs.size() > max_nms_)
    {
      objs.resize(max_nms_);
    }
    std::vector<float> v_area(objs.size());
    for (size_t i = 0; i < objs.size(); i++)
    {
      v_area[i] = (objs[i].x2 - objs[i].x1 + 1) * (objs[i].y2 - objs[i].y1 + 1);
    }
    for (size_t i = 0; i < objs.size(); i++)
    {
      for (size_t j = i + 1; j < objs.size();)
      {
        float xx1 = std::fmax(objs[i].x1, objs[j].x1);
        float yy1 = std::fmax(objs[i].y1, objs[j].y1);
        float xx2 = std::fmin(objs[i].x2, objs[j].x2);
        float yy2 = std::fmin(objs[i].y2, objs[j].y2);
        float w = std::fmax(0, xx2 - xx1 + 1);
        float h = std::fmax(0, yy2 - yy1 + 1);
        float inter = w * h;
        float ovr = inter / (v_area[i] + v_area[j] - inter);
        if (ovr >= iou_thresh_)
        {
          objs.erase(objs.begin() + j);
          v_area.erase(v_area.begin() + j);
        }
        else
        {
          j++;
        }
      }
    }
  }

  bool Yolo::BuildEngineFromONNX()
  {
    LOG(INFO) << "Engine will be built from onnx.";
    auto builder = make_shared(nvinfer1::createInferBuilder(logger_));
    if (!builder)
    {
      LOG(ERROR) << "Failed to create builder.";
      return false;
    }
    // auto config = make_shared(builder->createBuilderConfig());
    const auto explicit_batch = 1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
    auto network = make_shared(builder->createNetworkV2(explicit_batch));
    if (!network)
    {
      LOG(ERROR) << "Failed to create network.";
      return false;
    }
    auto config = make_shared(builder->createBuilderConfig());
    if (!config)
    {
      LOG(ERROR) << "Failed to create builder config.";
      return false;
    }
    auto parser = make_shared(nvonnxparser::createParser(*network, logger_));
    if (!parser)
    {
      LOG(ERROR) << "Failed to create parser.";
      return false;
    }

    if (!parser->parseFromFile(model_file_path_.c_str(), 1))
      LOG(ERROR) << "Failed to parse " << model_file_path_;
    if (builder->platformHasFastFp16())
    {
      LOG(INFO) << "Platform supports fp16, fp16 is enabled.";
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
    }
    else
    {
      LOG(INFO) << "FP16 is not supported on this platform, fp32 is enabled.";
      config->setFlag(nvinfer1::BuilderFlag::kTF32);
    }

    size_t free, total;
    cudaMemGetInfo(&free, &total);
    LOG(INFO) << "GPU memory total: " << (total >> 20) << "MB, free: " << (free >> 20) << "MB.";
    LOG(INFO) << "Max workspace size will use all of free GPU memory.";
    config->setMaxWorkspaceSize(free >> 1);
    auto profileStream = samplesCommon::makeCudaStream();
    config->setProfileStream(*profileStream);
    auto model_data = make_shared(builder->buildSerializedNetwork(*network, *config));
    FILE *f = fopen(model_cache_path_.c_str(), "wb");
    fwrite(model_data->data(), 1, model_data->size(), f);
    fclose(f);
    LOG(INFO) << "File has be written into cache.";
    return true;
  }

  void Yolo::BuildEngineFromCache()
  {
    LOG(INFO) << "Engine will be built from cache.";
    auto load_file = [&](const std::string &file) -> std::vector<unsigned char>
    {
      std::ifstream in(file, std::ios::in | std::ios::binary);
      if (!in.is_open())
        return {};
      in.seekg(0, std::ios::end);
      auto length = in.tellg();
      std::vector<uint8_t> data;
      if (length > 0)
      {
        in.seekg(0, std::ios::beg);
        data.resize(length);
        in.read((char *)&data[0], length);
      }
      in.close();
      return data;
    };
    auto engine_data = load_file(model_cache_path_);
    runtime_ = nvinfer1::createInferRuntime(logger_);
    engine_ = runtime_->deserializeCudaEngine(engine_data.data(), engine_data.size());
  }

} // namespace srm::nn