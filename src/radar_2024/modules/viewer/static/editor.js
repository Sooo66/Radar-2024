var editorArea = document.getElementById('text');
var editor = CodeMirror.fromTextArea(editorArea, {
  lineNumbers: true,
  mode: "text/x-toml",
  lint: true,
  vimMode: true
});
editor.on('change', () => {
});
CodeMirror.commands.save = function () {
  var content = editor.getValue(); // 获取CodeMirror的内容
  // 发送HTTP请求保存数据
  fetch("/save", {
    method: "POST",
    headers: {
      "Content-Type": "application/x-www-form-urlencoded"
    },
    body: "content=" + encodeURIComponent(content)
  })
    .then(response => response.json())
    .then(data => {
      console.log(data); // 处理返回的响应数据
    })
    .catch(error => {
      console.error("保存失败:", error);
    });
};

var editorCon = document.getElementById('left')
var editorResize = new ResizeObserver(entries => {
  for (let entry of entries) {
    const cr = entry.contentRect;
    editorArea.style.width = cr.width;
    editorArea.style.height = cr.height;
  }
}).observe(editorCon); 