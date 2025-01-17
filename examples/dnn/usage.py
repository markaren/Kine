import torch
from net import InverseKinematicsNet

model = InverseKinematicsNet()
model.load_state_dict(torch.load("crane3r_model.pth", weights_only=True))
model.eval()

# Test the model
test_target = torch.tensor([5.55, 7.4, 3.14])
predicted_angles = model(test_target.unsqueeze(0)).detach().numpy()

# Print the results
print("Input Position:", test_target.numpy())
print("Predicted Joint Angles (in degrees):", predicted_angles)
