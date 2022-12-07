from collections.abc import Sequence


"""
91-class COCO labels
`unlabled` (id = 0) is replaced with `head` for CrowdHuman
These are different from the default 80-class COCO labels used by YOLO
"""
_label_map = (
    'car',
    'bus',
    'truck',
    'motorbike',
    'person'
)


def get_label_name(class_id):
    """Look up label name given a class ID."""
    return _label_map[class_id]

def get_labels_all():
    """Returns the labels map"""
    # labels = [_label_map[i] for i in range(0,len(_label_map))]
    return _label_map

def set_label_map(label_map):
    """Set label name mapping from class IDs.

    Parameters
    ----------
    label_map : sequence
        A sequence of label names.
        The index of each label determines its class ID.
    """
    assert isinstance(label_map, Sequence)
    assert len(label_map) > 0
    global _label_map
    _label_map = tuple(label_map)
