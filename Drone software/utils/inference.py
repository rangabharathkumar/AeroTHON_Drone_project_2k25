import threading
import time
import torch
import numpy as np

class InferenceEngine:
    def __init__(self, model_loader):
        self.model_loader = model_loader
        self.results = {}
        self.running = False
        self.threads = {}
        self.locks = {}
        self.last_inference_time = {}
        
    def start_inference(self, model_names):
        self.running = True
        
        for model_name in model_names:
            self.results[model_name] = None
            self.locks[model_name] = threading.Lock()
            self.last_inference_time[model_name] = 0
            
            thread = threading.Thread(target=self._inference_loop, args=(model_name,))
            thread.daemon = True
            thread.start()
            self.threads[model_name] = thread
            
    def _inference_loop(self, model_name):
        model = self.model_loader.get_model(model_name)
        if model is None:
            print(f"Failed to get model: {model_name}")
            return
            
        while self.running:
            frame = self.frame_queue.get() if hasattr(self, 'frame_queue') else None
            
            if frame is None:
                time.sleep(0.01)
                continue
                
            try:
                start_time = time.time()
                results = model(frame)
                
                with self.locks[model_name]:
                    self.results[model_name] = results
                    self.last_inference_time[model_name] = time.time()
                    
                proc_time = time.time() - start_time
                if proc_time < 0.1:  # If processing is fast, add a small delay
                    time.sleep(0.1 - proc_time)
                    
            except Exception as e:
                print(f"Error in inference for {model_name}: {e}")
                time.sleep(0.1)
                
    def set_frame(self, frame):
        if hasattr(self, 'frame_queue'):
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except:
                    pass
            self.frame_queue.put(frame)
        else:
            import queue
            self.frame_queue = queue.Queue(maxsize=1)
            self.frame_queue.put(frame)
            
    def get_results(self, model_name):
        with self.locks.get(model_name, threading.Lock()):
            return self.results.get(model_name, None)
            
    def stop(self):
        self.running = False
        for thread in self.threads.values():
            if thread and thread.is_alive():
                thread.join(timeout=1.0)
                
    def run_inference(self, frame, model_name):
        model = self.model_loader.get_model(model_name)
        if model is None:
            return None
            
        try:
            results = model(frame)
            return results
        except Exception as e:
            print(f"Error in direct inference for {model_name}: {e}")
            return None
