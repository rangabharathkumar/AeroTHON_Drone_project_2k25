import tensorflow as tf
from tensorflow.keras.models import Model
from tensorflow.keras.layers import Input, Conv2D, BatchNormalization, Activation, MaxPooling2D, GlobalAveragePooling2D, Dense, Dropout, Add, Concatenate
from tensorflow.keras.regularizers import l2
from tensorflow.keras.initializers import he_normal

def residual_block(x, filters, kernel_size=3, stride=1, weight_decay=5e-4):
    """Enhanced residual block with improved feature extraction"""
    shortcut = x
    
    # First convolution
    x = Conv2D(filters, kernel_size, strides=stride, padding='same',
               kernel_initializer=he_normal(),
               kernel_regularizer=l2(weight_decay))(x)
    x = BatchNormalization()(x)
    x = Activation('relu')(x)
    
    # Second convolution
    x = Conv2D(filters, kernel_size, padding='same',
               kernel_initializer=he_normal(),
               kernel_regularizer=l2(weight_decay))(x)
    x = BatchNormalization()(x)
    
    # Shortcut connection
    if stride != 1 or shortcut.shape[-1] != filters:
        shortcut = Conv2D(filters, 1, strides=stride, padding='same',
                         kernel_initializer=he_normal(),
                         kernel_regularizer=l2(weight_decay))(shortcut)
        shortcut = BatchNormalization()(shortcut)
    
    x = Add()([x, shortcut])
    x = Activation('relu')(x)
    return x

def attention_block(x, filters):
    """Squeeze-and-Excitation attention block"""
    se = GlobalAveragePooling2D()(x)
    se = Dense(filters // 8, activation='relu')(se)
    se = Dense(filters, activation='sigmoid')(se)
    return tf.keras.layers.Multiply()([x, se])

def make_enhanced_emergency_net(H, W, C):
    """Create enhanced EmergencyNet model with improved architecture"""
    input_shape = [H, W, 3]
    inp = Input(shape=input_shape)
    
    # Initial convolution
    x = Conv2D(32, (7, 7), strides=2, padding='same',
               kernel_initializer=he_normal(),
               kernel_regularizer=l2(5e-4))(inp)
    x = BatchNormalization()(x)
    x = Activation('relu')(x)
    x = MaxPooling2D(pool_size=(3, 3), strides=2, padding='same')(x)
    
    # Enhanced feature extraction blocks
    x = residual_block(x, 64)
    x = attention_block(x, 64)
    x = MaxPooling2D()(x)
    
    x = residual_block(x, 128)
    x = attention_block(x, 128)
    x = MaxPooling2D()(x)
    
    x = residual_block(x, 256)
    x = attention_block(x, 256)
    x = MaxPooling2D()(x)
    
    x = residual_block(x, 512)
    x = attention_block(x, 512)
    
    # Global average pooling
    x = GlobalAveragePooling2D()(x)
    
    # Final classification layer
    x = Dense(512, activation='relu')(x)
    x = Dropout(0.5)(x)
    cls = Dense(C, activation='softmax', name='class_branch')(x)
    
    return inp, cls 