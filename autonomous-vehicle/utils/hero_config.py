#hero_config.py

hero_config = {
    "blueprint": "vehicle.mini.cooper_s",
    "spawn_points": [],
    "sensors": {
        # Front camera
        "camera_front": {
            "type": "sensor.camera.rgb",
            "transform": "1,0.0,1.4,0.0,0.0,0.0",
            "image_size_x": "400",
            "image_size_y": "300",
            "fov": "90"
        },
        
        # # Rear camera
        # "camera_rear": {
        #     "type": "sensor.camera.rgb",
        #     "transform": "-2.0,0.0,1.4,0.0,0.0,180.0",
        #     "image_size_x": "800",
        #     "image_size_y": "600",
        #     "fov": "90"
        # },
        
        # # Left camera
        # "camera_left": {
        #     "type": "sensor.camera.rgb",
        #     "transform": "0.0,-1.5,1.4,0.0,0.0,-90.0",
        #     "image_size_x": "800",
        #     "image_size_y": "600",
        #     "fov": "90"
        # },
        
        # # Right camera
        # "camera_right": {
        #     "type": "sensor.camera.rgb",
        #     "transform": "0.0,1.5,1.4,0.0,0.0,90.0",
        #     "image_size_x": "800",
        #     "image_size_y": "600",
        #     "fov": "90"
        # },
        
        # Top-down camera
        "camera_top": {
            "type": "sensor.camera.rgb",
            "transform": "0.0,0.0,10.0,0.0,-90.0,0.0",  # 5m up, pointing down
            "image_size_x": "400",
            "image_size_y": "300",
            "fov": "90"
        }
    }
}