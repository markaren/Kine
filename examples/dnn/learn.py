import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from sklearn.model_selection import train_test_split

# Save the trained model
model_name = "crane3r"

# Check if a GPU is available, otherwise use the CPU
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print(f"Using device: {device}")

# Load data
positions = np.loadtxt(f"training/{model_name}/positions.csv", delimiter=",")
angles = np.loadtxt(f"training/{model_name}/values.csv", delimiter=",")

# Split data into training and validation sets
X_train, X_val, y_train, y_val = train_test_split(positions, angles, test_size=0.2, random_state=42)

# Convert to PyTorch tensors
X_train = torch.tensor(X_train, dtype=torch.float32).to(device)
y_train = torch.tensor(y_train, dtype=torch.float32).to(device)
X_val = torch.tensor(X_val, dtype=torch.float32).to(device)
y_val = torch.tensor(y_val, dtype=torch.float32).to(device)

# Instantiate the model, define loss and optimizer
from net import InverseKinematicsNet

model = InverseKinematicsNet().to(device)
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# Train the model
epochs = 50000
for epoch in range(epochs):
    model.train()
    optimizer.zero_grad()

    outputs = model(X_train)

    loss = criterion(outputs, y_train)

    loss.backward()
    optimizer.step()

    # Validate the model
    model.eval()
    with torch.no_grad():
        val_outputs = model(X_val)
        val_loss = criterion(val_outputs, y_val)

    validation_loss = val_loss.item()
    if epoch % 1000 == 0:
        print(f"Epoch {epoch}, Training Loss: {loss.item()}, Validation Loss: {validation_loss}")

    if validation_loss < 0.1:
        print(f"Achieved suitable training performance after {epoch} epochs ({validation_loss}). Aborting training early..")
        break


# Export the model in pytorch format
torch.save(model.state_dict(), f"{model_name}.pth")

# Export the model to ONNX format
onnx_file_path = f"{model_name}.onnx"
torch.onnx.export(
    model,  # Model to export
    torch.tensor(np.zeros(3), dtype=torch.float32).unsqueeze(0).to(device),  # Dummy input tensor
    onnx_file_path,  # Path to save the ONNX model
    export_params=True,  # Store trained weights in the model
    opset_version=11,  # ONNX opset version
    do_constant_folding=True,  # Optimize constant folding for inference
    input_names=["input"],  # Name of the input layer
    output_names=["output"]  # Name of the output layer
)
