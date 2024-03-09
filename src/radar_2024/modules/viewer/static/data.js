var videoCon = document.getElementById('main')
var videoArea = document.getElementById('video')
var videoResize = new ResizeObserver(entries => {
  for (let entry of entries) {
    const cr = entry.contentRect;
    videoArea.width = cr.width;
    videoArea.height = cr.height;
    console.log(videoArea.width)
    console.log(videoArea.height)
  }
}).observe(videoCon); 