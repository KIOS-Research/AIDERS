from .general import download, Path


  # Download labels
#
segments = False  # segment or box labels
dir = Path("./dataset/coco/labels")  # dataset root dir
url = 'https://github.com/ultralytics/yolov5/releases/download/v1.0/'
urls = [url + ('coco2017labels-segments.zip' if segments else 'coco2017labels.zip')]  # labels
download(urls, dir=dir.parent)