from gr3_compute_side.high_level_planner.schedule.schedule import object_cls_dic

def inference():
    # top, left, width, height in pixels
    x = 80
    y = 120
    w = 60
    h = 40
    pred = "f1"
    cls = object_cls_dic[pred]
    return [x, y, w, h, cls]