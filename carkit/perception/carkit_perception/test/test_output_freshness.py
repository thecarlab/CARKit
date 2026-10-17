"""Exercise the production publication method without CUDA/model imports."""
import ast
from pathlib import Path
from types import SimpleNamespace


def test_single_inference_is_consumed_once_and_stale_output_is_discarded():
    source = Path(__file__).parents[1]/'carkit_perception/perception_2d_node.py'
    tree = ast.parse(source.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Perception2DNode')
    method = next(n for n in node.body if isinstance(n, ast.FunctionDef) and n.name == 'publish_latest_results')
    scope = {'time':SimpleNamespace(monotonic=lambda:10.)}
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), scope)
    detections, images = [], []
    target = SimpleNamespace(output_received_at=9.9, output_timeout_sec=.5,
        latest_detection_output='one frame', latest_raw_inference_image=None,
        latest_compressed_inference_image='one jpeg',
        detection_pub=SimpleNamespace(publish=detections.append),
        compressed_image_pub=SimpleNamespace(publish=images.append, get_subscription_count=lambda:1))
    for _ in range(100):
        scope['publish_latest_results'](target)
    assert detections == ['one frame'] and images == ['one jpeg']
    target.latest_detection_output = 'expired frame'
    target.latest_compressed_inference_image = 'expired jpeg'
    target.output_received_at = 9.
    scope['publish_latest_results'](target)
    assert detections == ['one frame'] and images == ['one jpeg']
    assert target.latest_detection_output is None
