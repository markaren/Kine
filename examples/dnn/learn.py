import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
from sklearn.model_selection import train_test_split

# Load data
positions = np.loadtxt("training/positions.csv", delimiter=",")
angles = np.loadtxt("training/angles.csv", delimiter=",")

# Split data into training and validation sets
X_train, X_val, y_train, y_val = train_test_split(positions, angles, test_size=0.2, random_state=42)

# Convert to PyTorch tensors
X_train = torch.tensor(X_train, dtype=torch.float32)
y_train = torch.tensor(y_train, dtype=torch.float32)
X_val = torch.tensor(X_val, dtype=torch.float32)
y_val = torch.tensor(y_val, dtype=torch.float32)

# Define the neural network
class InverseKinematicsNet(nn.Module):
    def __init__(self):
        super(InverseKinematicsNet, self).__init__()
        self.fc1 = nn.Linear(3, 64)
        self.fc2 = nn.Linear(64, 128)
        self.fc3 = nn.Linear(128, 64)
        self.fc4 = nn.Linear(64, 3)

    def forward(self, x):
        x = torch.relu(self.fc1(x))
        x = torch.relu(self.fc2(x))
        x = torch.relu(self.fc3(x))
        return self.fc4(x)

# Instantiate the model, define loss and optimizer
model = InverseKinematicsNet()
criterion = nn.MSELoss()
optimizer = optim.Adam(model.parameters(), lr=0.001)

# Train the model
epochs = 1000
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

    if epoch % 10 == 0:
        print(f"Epoch {epoch}, Training Loss: {loss.item()}, Validation Loss: {val_loss.item()}")

# Save the trained model
torch.save(model.state_dict(), "inverse_kinematics_model.pth")

# Test the model
test_target = torch.tensor([5.55, 7.4, 3.14])
predicted_angles = model(test_target.unsqueeze(0)).detach().numpy()

# Print the results
print("Input Position:", test_target.numpy())
print("Predicted Joint Angles (in radians):", predicted_angles)
