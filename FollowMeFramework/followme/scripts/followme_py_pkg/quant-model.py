import torch
import torch.nn as nn
import torch.quantization
from torchvision import datasets, transforms
from followme_py_pkg.Reidentificator import Reidentificator
import os

# Assume the model is already trained and we load the state_dict
#pkg_dir_name = '/' + os.path.join(*__file__.split('/')[:-2])
#weights_dir = pkg_dir_name + "../../weights"
#mmt_weights = os.path.join(weights_dir, "old_pytorch_resnet_ibn_REID_feat256_train_msmt17.pth")
mmt_weights = "old_pytorch_resnet_ibn_REID_feat256_train_msmt17.pth"
calibration_filename = "calibration_default.pkl"
#calibration_dir = pkg_dir_name + "/calibrations"
calibration_path = calibration_filename
class_model = Reidentificator(class_target="person", display_img=False, model_weights=mmt_weights, calibration_path = calibration_path)
model = class_model.model_REID.module
checkpoint = torch.load(mmt_weights,map_location=torch.device('cpu'))
new_state_dict = {k[7:] if k.startswith('module.') else k: v for k, v in checkpoint['state_dict'].items()}
model.load_state_dict(new_state_dict, strict=False)

model.eval()
print(model)


# fuse_modules = [
#     ['base.0', 'base.1', 'base.2'],  # Fuse conv1, bn1, relu
#     ['base.4.0.conv1', 'base.4.0.bn1', 'base.4.0.relu'],  # Fuse conv1, bn1, relu in first bottleneck
#     ['base.4.0.conv2', 'base.4.0.bn2'],  # Fuse conv2, bn2 in first bottleneck
#     ['base.4.0.conv3', 'base.4.0.bn3'],  # Fuse conv3, bn3 in first bottleneck
#     # Add more layers if needed
# ]
# # Fuse the model's layers
# model_fused = torch.quantization.fuse_modules(model, fuse_modules, inplace=True)

# model_fused.eval()

# # Set the quantization configuration
model.qconfig = torch.quantization.get_default_qconfig('fbgemm')

# Prepare the model for static quantization
model_prepared = torch.quantization.prepare(model, inplace=True)

# Convert to quantized model
model_quantized = torch.quantization.convert(model_prepared, inplace=True)

# Save the quantized model
torch.save({
    'state_dict': model.state_dict(),
    'epoch': checkpoint['epoch'],
    'best_mAP': checkpoint['best_mAP']
}, 'QUANT_old_pytorch_resnet_ibn_REID_feat256_train_msmt17.pth.pth')

print("Model quantized and saved successfully.")