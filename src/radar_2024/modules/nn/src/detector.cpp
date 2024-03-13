#include <srm/nn/detector.h>
#include <srm/common/config.hpp>
#include <algorithm>
namespace srm::nn {

bool Detector::Initialize() {
  car_nn_.reset(CreateYolo("car"));
  armor_nn_.reset(CreateYolo("armor"));

  const std::string car_prefix = "nn.car";
  const std::string armor_prefix = "nn.armor";

  auto car_model_file = cfg.Get<std::string>({car_prefix, "model_path"});
  auto armor_model_file = cfg.Get<std::string>({armor_prefix, "model_path"});
  auto car_num_classes = cfg.Get<int>({car_prefix, "class_num"});
  auto armor_num_classes = cfg.Get<int>({armor_prefix, "class_num"});

  if (!car_nn_->Initialize(car_model_file, car_num_classes)) {
    LOG(ERROR) << "Faield to initialize car network";
    return false;
  }
  if (!armor_nn_->Initialize(armor_model_file, armor_num_classes)) {
    LOG(ERROR) << "Failed to initialize armor network";
    return false;
  }
  return true;
}

bool Detector::Run(std::vector<cv::Mat> REF_IN image_list, std::vector<Armor> REF_OUT armor_list) {
  for (int i = 0; i < image_list.size(); i ++) {
    const auto& image = image_list[i];
    auto cars = car_nn_->Infer(image);
    for (auto&& car: cars) {
      ROI roi;
      auto roi_mat = image(cv::Rect(car.x1, car.y1, car.x2 - car.x1, car.y2 - car.y1));
      roi.top_left = {car.x1, car.y1};
      roi.width = car.x2 - car.x1;
      roi.height = car.y2 - car.y1;
      std::vector<Objects> objs = armor_nn_->Infer(roi_mat);
      auto obj = std::max_element(objs.begin(), objs.end(),
                                  [](const Objects& a, const Objects &b) { return a.prob < b.prob; }
                                 );
      if (obj != objs.end()) {
        Armor armor;
        armor.id = obj->cls % 6 + 1;
        armor.color = static_cast<Color>(obj->cls / 6 + 1);
        armor.roi = std::move(roi);
        armor.prob = obj->prob;
        armor.pts = std::array<cv::Point2f, 2>{ {cv::Point2f(obj->x1, obj->y1), cv::Point2f(obj->x2, obj->y2)} };
        armor.source = i;
        armor_list.push_back(std::move(armor));
      }
    }
  }

  // ArmorFilter
  std::sort(armor_list.begin(), armor_list.end(), [](const Armor& a, const Armor& b) {
    return a.prob > b.prob;
  });
  armor_list.erase(std::unique(armor_list.begin(), armor_list.end(), [](const Armor& a, const Armor& b) {
    return a.id == b.id;
  }), armor_list.end());
  return true;
}

} // namespace srm::n