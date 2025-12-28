import torch
import os
import time

class ModelLoader:
    def __init__(self, models_dir="./models"):
        self.models_dir = models_dir
        self.models = {}
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")
        
    def load_model(self, model_name):
        try:
            model_path = os.path.join(self.models_dir, f"{model_name}.pt")
            if not os.path.exists(model_path):
                print(f"Model not found: {model_path}")
                return False
            
            start_time = time.time()
            model = torch.hub.load('ultralytics/yolov8', 'custom', path=model_path)
            model.to(self.device)
            
            # Set inference parameters
            model.conf = 0.25  # confidence threshold
            model.iou = 0.45   # NMS IoU threshold
            
            self.models[model_name] = model
            print(f"Loaded {model_name} in {time.time() - start_time:.2f} seconds")
            return True
            
        except Exception as e:
            print(f"Error loading model {model_name}: {e}")
            return False
    
    def get_model(self, model_name):
        if model_name not in self.models:
            success = self.load_model(model_name)
            if not success:
                return None
                
        return self.models[model_name]
