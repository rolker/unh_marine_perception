import blobconverter
import sys
import os

def convert():
    blob_path = blobconverter.from_onnx(
        model="ewasr_resnet18.onnx",
        output_dir="../config",
        shaves=6,
        optimizer_params=[
            "--mean_values=[123.675,116.28,103.53]",
            "--scale_values=[58.395,57.12,57.375]",
            "--reverse_input_channels"
        ],
        version="2022.1",
        data_type="FP16"
    )
    
    # Rename if necessary/predictable
    # blobconverter usually names it <model_name>_openvino_<version>_<shaves>shave.blob
    # We want ewasr_resnet18.blob
    
    expected_name = "ewasr_resnet18.blob"
    target_path = os.path.join("../config", expected_name)
    
    # Find the generated file
    if os.path.exists(blob_path):
        os.rename(blob_path, target_path)
        print(f"Successfully converted and renamed to {target_path}")
    else:
        print(f"Conversion returned path {blob_path} but file not found.")
        sys.exit(1)

if __name__ == "__main__":
    convert()
