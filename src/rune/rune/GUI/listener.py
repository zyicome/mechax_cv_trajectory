import datetime

import numpy

from rune.linear_calculation import ContourInfo


class Listener:
    def __init__(self):
        self.image_width = None
        self.image_height = None
        self.all_contours: list[ContourInfo] = []
        self.selected_by_area: list[ContourInfo] = []
        self.selected_by_hu: list[ContourInfo] = []
        self.predicted: numpy.ndarray = None
        self.delta_time = -1
        self.last_update = datetime.datetime.now()

    def update_time(self):
        now = datetime.datetime.now()
        self.delta_time = (now - self.last_update).total_seconds()
        self.last_update = now

    def clean(self):
        self.all_contours = []
        self.selected_by_area = []
        self.selected_by_hu = []
        self.predicted = None


def listen(func: callable):
    def to_gray_listened(image: numpy.ndarray) -> numpy.ndarray:
        listener.update_time()
        listener.image_width, listener.image_height = image.shape[1], image.shape[0]
        return func(image)

    def filter_by_area_listened(contours: list[ContourInfo]) -> list[ContourInfo]:
        listener.all_contours = contours
        listener.selected_by_area = func(contours)
        return listener.selected_by_area

    def filter_by_hu_listened(contours: list[ContourInfo]) -> list[ContourInfo]:
        listener.selected_by_hu = func(contours)
        return listener.selected_by_hu

    def predict(vector: numpy.ndarray, radian: float):
        result = func(vector, radian)
        listener.predicted = result
        return result

    if func.__name__ == "to_gray":
        return to_gray_listened
    if func.__name__ == "filter_by_area":
        return filter_by_area_listened
    if func.__name__ == "filter_by_hu":
        return filter_by_hu_listened
    if func.__name__ == "predict":
        return predict
    return func


listener = Listener()
