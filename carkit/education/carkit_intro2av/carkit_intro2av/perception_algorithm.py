# CARKit learning annotation: implements the behavior described by this file's package and module.
"""Student-owned perception algorithm."""

from dataclasses import dataclass, field

from carkit_perception_msgs.msg import (
    YoloDetection2D,
    YoloTrafficLightDetection2D,
)
from sensor_msgs.msg import CameraInfo, CompressedImage


@dataclass(frozen=True)
class PerceptionConfig:
    """Detector settings supplied by the ROS node."""

    minimum_confidence: float
    image_size: int
    model_path: str


@dataclass
class PerceptionResult:
    """Typed output consumed by the common ROS publication wrapper."""

    detections: list[YoloDetection2D] = field(default_factory=list)
    traffic_lights: list[YoloTrafficLightDetection2D] = field(
        default_factory=list
    )
    preview: CompressedImage | None = None


def process_image(
    image: CompressedImage,
    camera_info: CameraInfo | None,
    config: PerceptionConfig,
) -> PerceptionResult:
    """Run detection for one compressed camera frame."""
    # TODO(Intro2AV): Decode image.data, run your model or classical vision
    # pipeline, and return typed detections. Keep confidence in [0, 1] and
    # bounding boxes in source-image pixel coordinates. An annotated JPEG can
    # be returned in result.preview for the WebUI.
    del image, camera_info, config
    return PerceptionResult()
