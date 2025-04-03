import flwr as fl
import tensorflow as tf


from hwcounter import Timer, count, count_end
from time import sleep
from math import sqrt

# Load model and data (MobileNetV2, CIFAR-10)
model = tf.keras.applications.MobileNetV2((32, 32, 3), classes=10, weights=None)
model.compile("adam", "sparse_categorical_crossentropy", metrics=["accuracy"])
(x_train, y_train), (x_test, y_test) = tf.keras.datasets.cifar10.load_data()

# Define Flower client
class CifarClient(fl.client.NumPyClient):
  def get_parameters(self, config):
    return model.get_weights()

  def fit(self, parameters, config):
    model.set_weights(parameters)
    model.fit(x_train, y_train, epochs=1, batch_size=32)
    return model.get_weights(), len(x_train), {}

  def evaluate(self, parameters, config):
    model.set_weights(parameters)
    loss, accuracy = model.evaluate(x_test, y_test)
    return loss, len(x_test), {"accuracy": accuracy}

# Start Flower client
start = count()
fl.client.start_numpy_client(server_address="127.0.0.1:8080", client=CifarClient())
elapsed = count_end() - start
print(f'elapsed cycles: {elapsed}')

#with Timer() as t:
#  fl.client.start_numpy_client(server_address="127.0.0.1:8080", client=CifarClient())
#print(f'elapsed cycles: {t.cycles}')
