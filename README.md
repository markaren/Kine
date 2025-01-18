# Kine (work in progress)

>A simple kinematics library


Kine is a simple kinematics library, 
allowing users to describe generic kinematics chains in a simple an intuitive way.

```cpp
kine::Kine kine = kine::KineBuilder()
                  .addRevoluteJoint(Vector3::Y(), {-90.f, 90.f})
                  .addLink(Vector3::Y() * 4.2)
                  .addRevoluteJoint(Vector3::X(), {-80.f, 0.f})
                  .addLink(Vector3::Z() * 7)
                  .addRevoluteJoint(Vector3::X(), {40.f, 140.f})
                  .addLink(Vector3::Z() * 5.2)
                  .build();
```

From this, the Forward Kinematics (FK) and Inverse Kinematics is easily available.

## Inverse Kiematics
For solving the IK the library supports:

- Cyclic Coordinate Descent
- Damped Least Squared
- Deep Neural Network (DNN)


### Deep learning

The deep learning module is based on PyTorch and ONNX runtime.

1. Describe the model and generate dataset in C++
2. Train the DNN using PyTorch and export in ONNX format
3. Load the model in C++ with onnxruntime


#### Python environment

The DNN module is located in `examples/dnn`.

Install PyTorch per [instructions](https://pytorch.org/)
Install further libraries listed in `requirements.txt`

#### Setting up onnxruntime

1. Download and extract binaries from https://github.com/microsoft/onnxruntime 
2. Add an ENV variable `ONNX_RUNTIME_DIR` pointing to the extracted folder. 
3. Add the shared library `onnxruntime.xx` to PATH.
