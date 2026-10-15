import dataclasses

import einops
import numpy as np

from openpi import transforms
from openpi.models import model as _model


def make_libero_example() -> dict:
    """Creates a random input example for the Libero policy."""
    return {
        "observation/state": np.random.rand(8),
        "observation/image": np.random.randint(256, size=(224, 224, 3), dtype=np.uint8),
        "observation/wrist_image": np.random.randint(256, size=(224, 224, 3), dtype=np.uint8),
        "prompt": "do something",
    }


def _parse_image(image) -> np.ndarray:
    image = np.asarray(image)
    if np.issubdtype(image.dtype, np.floating):
        image = (255 * image).astype(np.uint8)
    if image.shape[0] == 3:
        image = einops.rearrange(image, "c h w -> h w c")
    return image


@dataclasses.dataclass(frozen=True)
class CustomInputs(transforms.DataTransformFn):
    """
    This class is used to convert inputs to the model to the expected format. It is used for both training and inference.

    Dataset keys:
        observation.images.overhead_left
        observation.images.left_wrist
        observation.images.right_wrist
        observation.state      
        action                  
        task                    
    """

    # Determines which model will be used.
    # Do not change this for your own dataset.
    model_type: _model.ModelType

    def __call__(self, data: dict) -> dict:
      
        base_image = _parse_image(
            data["observation.images.overhead_left"]
        )
        left_wrist_image = _parse_image(
            data["observation.images.left_wrist"]
        )
        right_wrist_image = _parse_image(
            data["observation.images.right_wrist"]
        )

        inputs = {
            
            "state": data["observation.state"],

            "image": {
                "base_0_rgb": base_image,
                "left_wrist_0_rgb": left_wrist_image,
                "right_wrist_0_rgb": right_wrist_image,
            },

          
            "image_mask": {
                "base_0_rgb": np.True_,
                "left_wrist_0_rgb": np.True_,
                "right_wrist_0_rgb": np.True_,
            },
        }

        
        if "actions" in data:
            inputs["actions"] = data["actions"]

        
        if "task" in data:
            inputs["prompt"] = data["task"]


        elif "prompt" in data:
            inputs["prompt"] = data["prompt"]

        return inputs





@dataclasses.dataclass(frozen=True)
class CustomOutputs(transforms.DataTransformFn):
    """
    This class is used to convert outputs from the model back the the dataset specific format. It is
        used for inference only.
    
    For your own dataset, you can copy this class and modify the action dimension based on the comments below.
    """

    def __call__(self, data: dict) -> dict:
        return {
            "actions": np.asarray(data["actions"][..., :16])
        }

