import tensorflow as tf
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from tensorflow.keras.callbacks import ModelCheckpoint, EarlyStopping, ReduceLROnPlateau, TensorBoard
from tensorflow.keras.optimizers import Adam
from enhanced_model import make_enhanced_emergency_net
from augment import create_augmentations
import os
import datetime

def train_model(data_dir, img_size=224, batch_size=32, epochs=100):
    # Create data generators with augmentation
    train_datagen = ImageDataGenerator(
        preprocessing_function=lambda x: (x/127.5) - 1,
        validation_split=0.2,
        horizontal_flip=True,
        rotation_range=20,
        width_shift_range=0.2,
        height_shift_range=0.2,
        zoom_range=0.2
    )

    train_generator = train_datagen.flow_from_directory(
        data_dir,
        target_size=(img_size, img_size),
        batch_size=batch_size,
        class_mode='categorical',
        subset='training'
    )

    val_generator = train_datagen.flow_from_directory(
        data_dir,
        target_size=(img_size, img_size),
        batch_size=batch_size,
        class_mode='categorical',
        subset='validation'
    )

    # Create model
    input_layer, output_layer = make_enhanced_emergency_net(img_size, img_size, 5)  # 5 classes
    model = tf.keras.Model(inputs=input_layer, outputs=output_layer)

    # Compile model with improved optimizer settings
    optimizer = Adam(learning_rate=0.001, beta_1=0.9, beta_2=0.999, epsilon=1e-07)
    model.compile(
        optimizer=optimizer,
        loss='categorical_crossentropy',
        metrics=['accuracy']
    )

    # Create callbacks
    log_dir = "logs/fit/" + datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
    callbacks = [
        ModelCheckpoint(
            'enhanced_emergency_net.h5',
            monitor='val_accuracy',
            save_best_only=True,
            mode='max'
        ),
        EarlyStopping(
            monitor='val_loss',
            patience=10,
            restore_best_weights=True
        ),
        ReduceLROnPlateau(
            monitor='val_loss',
            factor=0.2,
            patience=5,
            min_lr=1e-6
        ),
        TensorBoard(
            log_dir=log_dir,
            histogram_freq=1
        )
    ]

    # Train model
    history = model.fit(
        train_generator,
        validation_data=val_generator,
        epochs=epochs,
        callbacks=callbacks
    )

    return model, history

if __name__ == "__main__":
    # Set parameters
    DATA_DIR = "data/AIDER"  # Path to your dataset
    IMG_SIZE = 224
    BATCH_SIZE = 32
    EPOCHS = 100

    # Train model
    model, history = train_model(DATA_DIR, IMG_SIZE, BATCH_SIZE, EPOCHS) 