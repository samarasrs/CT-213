from tensorflow.keras import layers, activations
from tensorflow.keras.models import Sequential


def make_lenet5():
    model = Sequential()

    # Todo: implement LeNet-5 model
    model.add(layers.Conv2D(filters=1, kernel_size=(1, 1), strides=(1, 1), input_shape=(32, 32, 1)))
    # 1 Conv2D
    model.add(layers.Conv2D(filters=6, kernel_size=(5, 5), strides=(1, 1), activation=activations.tanh))
    # 2 AveragePooling2D
    model.add(layers.AveragePooling2D(pool_size=(2, 2), strides=(2, 2)))
    # 3 Conv2D
    model.add(layers.Conv2D(filters=16, kernel_size=(5, 5), strides=(1, 1), activation=activations.tanh))
    # 4 AveragePooling2D
    model.add(layers.AveragePooling2D(pool_size=(2, 2), strides=(2, 2)))
    # 5 Conv2D
    model.add(layers.Conv2D(filters=120, kernel_size=(5, 5), strides=(1, 1), activation=activations.tanh))
    # 6 Dense (FC)
    model.add(layers.Flatten())
    model.add(layers.Dense(units=84, activation=activations.tanh))
    # 7 Dense (FC)
    model.add(layers.Dense(units=10, activation=activations.softmax))

    return model
