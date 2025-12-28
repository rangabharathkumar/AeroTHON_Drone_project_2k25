import cv2
import numpy as np
import time

class OverlayManager:
    def __init__(self):
        self.colors = {}
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        self.line_thickness = 2
        self.text_size = 0.5
        self.text_thickness = 1
        
    def _get_color(self, class_name):
        if class_name not in self.colors:
            # Generate a random color for this class
            self.colors[class_name] = (
                np.random.randint(0, 255),
                np.random.randint(0, 255),
                np.random.randint(0, 255)
            )
        return self.colors[class_name]
        
    def draw_predictions(self, frame, results, model_name=None):
        if results is None or len(results) == 0:
            return frame
            
        img = frame.copy()
        
        for i, result in enumerate(results):
            boxes = result.boxes
            
            for j, box in enumerate(boxes):
                # Get box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                
                # Get class and confidence
                cls_id = int(box.cls[0].item())
                conf = box.conf[0].item()
                
                # Get class name
                cls_name = result.names[cls_id]
                
                # Get color for this class
                color = self._get_color(cls_name)
                
                # Draw bounding box
                cv2.rectangle(img, (x1, y1), (x2, y2), color, self.line_thickness)
                
                # Prepare label text
                label = f"{cls_name}: {conf:.2f}"
                
                # Get text size
                (text_width, text_height), _ = cv2.getTextSize(
                    label, self.font, self.text_size, self.text_thickness
                )
                
                # Draw text background
                cv2.rectangle(
                    img, 
                    (x1, y1 - text_height - 5), 
                    (x1 + text_width, y1), 
                    color, 
                    -1
                )
                
                # Draw text
                cv2.putText(
                    img,
                    label,
                    (x1, y1 - 5),
                    self.font,
                    self.text_size,
                    (255, 255, 255),
                    self.text_thickness
                )
                
                # Add model name if provided
                if model_name:
                    cv2.putText(
                        img,
                        f"Model: {model_name}",
                        (10, 30),
                        self.font,
                        0.7,
                        (0, 255, 0),
                        2
                    )
        
        # Add timestamp
        timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        cv2.putText(
            img,
            timestamp,
            (10, img.shape[0] - 10),
            self.font,
            0.5,
            (255, 255, 255),
            1
        )
        
        return img
        
    def get_target_center(self, results, target_class="target"):
        if results is None or len(results) == 0:
            return None
            
        for result in results:
            boxes = result.boxes
            
            for box in boxes:
                # Get class
                cls_id = int(box.cls[0].item())
                cls_name = result.names[cls_id]
                
                # Check if this is our target class
                if cls_name == target_class:
                    # Get box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
                    
                    # Calculate center
                    center_x = (x1 + x2) // 2
                    center_y = (y1 + y2) // 2
                    
                    # Get confidence
                    conf = box.conf[0].item()
                    
                    return {
                        "center": (center_x, center_y),
                        "confidence": conf,
                        "bbox": (x1, y1, x2, y2)
                    }
                    
        return None
        
    def highlight_target(self, frame, target_info):
        if target_info is None:
            return frame
            
        img = frame.copy()
        center = target_info["center"]
        
        # Draw crosshair at center
        cv2.drawMarker(
            img, 
            center, 
            (0, 255, 0), 
            markerType=cv2.MARKER_CROSS, 
            markerSize=20, 
            thickness=2
        )
        
        # Draw circle around center
        cv2.circle(
            img,
            center,
            30,
            (0, 255, 0),
            2
        )
        
        # Add target text
        cv2.putText(
            img,
            f"TARGET FOUND: {target_info['confidence']:.2f}",
            (center[0] - 70, center[1] - 40),
            self.font,
            0.7,
            (0, 255, 0),
            2
        )
        
        return img
