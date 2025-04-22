## TensorBoard and PyTorch

Documentation: <br>
https://pytorch.org/tutorials/recipes/recipes/tensorboard_with_pytorch.html <br>
https://pytorch.org/tutorials/intermediate/tensorboard_tutorial.html <br>
https://www.youtube.com/watch?v=VJW9wU-1n18

### Installation
```shell
$ pip install torch torchvision
```

### Using TensorBoard in PyTorch
Let’s now try using TensorBoard with PyTorch! Before logging anything, we need to create a SummaryWriter instance.
```shell
$ import torch
$ from torch.utils.tensorboard import SummaryWriter
$ writer = SummaryWriter()
```
Writer will output to __./runs/__ directory by default.

### Log scalars
In machine learning, it’s important to understand key metrics such as loss and how they change during training. Scalar helps to save the loss value of each training step, or the accuracy after each epoch.

To log a scalar value, use add_scalar(tag, scalar_value, global_step=None, walltime=None). For example, lets create a simple linear regression training, and log loss value using add_scalar

```python
x = torch.arange(-5, 5, 0.1).view(-1, 1)
y = -5 * x + 0.1 * torch.randn(x.size())

model = torch.nn.Linear(1, 1)
criterion = torch.nn.MSELoss()
optimizer = torch.optim.SGD(model.parameters(), lr = 0.1)

def train_model(iter):
    for epoch in range(iter):
        y1 = model(x)
        loss = criterion(y1, y)
        writer.add_scalar("Loss/train", loss, epoch)
        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

train_model(10)
writer.flush()
```
Call flush() method to make sure that all pending events have been written to disk.

See torch.utils.tensorboard tutorials to find more TensorBoard visualization types you can log.

If you do not need the summary writer anymore, call close() method.
```python
writer.close()
```

### Installing TensorBoard

```shell
$ pip install tensorboard
$ tensorboard --logdir=runs
```
Go to the URL it provides OR to http://localhost:6006/

