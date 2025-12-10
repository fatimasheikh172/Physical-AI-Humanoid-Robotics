# Week 1: Synthetic Data Generation with Isaac Replicator

## Learning Objectives

By the end of this section, you will be able to:
- Use Isaac Replicator for synthetic dataset generation
- Apply domain randomization techniques to improve sim-to-real transfer
- Generate photorealistic synthetic datasets with ground truth annotations
- Validate synthetic dataset quality and accuracy

## Introduction to Isaac Replicator

Isaac Replicator is NVIDIA's synthetic data generation framework that enables the creation of photorealistic datasets with accurate ground truth annotations. It's an essential tool for training perception models when real-world data is scarce or expensive to collect.

## 1. Setting Up Isaac Replicator

### Basic Replicator Configuration

Here's an example script for setting up Isaac Replicator to generate synthetic datasets:

```python
# synthetic_data_generator.py
import omni.replicator.core as rep

# Initialize Isaac Replicator
rep.orchestrator._orchestrator._is_started = True

# Define domain randomization functions
def randomize_lighting():
    """Randomize lighting conditions in the scene"""
    lights = rep.create.light(
        light_type="distant",
        position=rep.distribution.uniform((-100, -100, 300), (100, 100, 500)),
        intensity=rep.distribution.uniform(300, 1000),
        color=rep.distribution.uniform((0.5, 0.5, 0.5), (1.0, 1.0, 1.0))
    )
    return lights

def randomize_materials():
    """Randomize materials for domain randomization"""
    # Create random materials
    brown_mats = rep.utils.get_usd_shade_mats_from_directory(
        "Omni/Kit/Albedo/Textures/Concrete", 
        num=100
    )
    
    gray_mats = rep.utils.get_usd_shade_mats_from_directory(
        "Omni/Kit/Albedo/Textures/Metal", 
        num=100
    )
    
    all_mats = brown_mats + gray_mats
    
    # Assign to objects
    with rep.trigger.on_frame(num_frames=50):
        rep.randomizer.scatter_2d(
            objects=rep.utils.get_usd_prims_from_directory("/World/Props"),
            target=rep.get.prims_from_path("/World/Ground"),
            in_range=((0, 0), (10, 10)),
            rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 3.14))
        )
        
        # Randomize materials
        rep.randomizer.material(
            inputs={"inputs:diffuse_texture": all_mats}
        )

def setup_camera_data_stream():
    """Configure camera for data capture"""
    # Create camera
    camera = rep.create.camera(
        focal_length=24,
        position=(0, -5, 2),
        look_at=(0, 0, 0)
    )
    
    # Set up RGB capture
    render_product = rep.create.render_product(
        camera,
        resolution=(640, 480)
    )
    
    # Register RGB annotator
    rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb_annotator.attach(render_product)
    
    # Register depth annotator
    depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")
    depth_annotator.attach(render_product)
    
    # Register semantic segmentation annotator
    seg_annotator = rep.AnnotatorRegistry.get_annotator("instance_segmentation")
    seg_annotator.attach(render_product)
    
    return camera

# Configure the replicator
with rep.new_layer():
    # Set up scene
    ground = rep.create.plane(semantics=[rep.semantics.SemanticLabel("ground", 0)])
    robot = rep.create.from_usd(
        "path/to/humanoid_robot.usd",
        position=(0, 0, 0.5),
        semantics=[rep.semantics.SemanticLabel("robot", 1)]
    )
    
    # Add obstacles with semantic labels
    obstacles = rep.create.cube(
        position=rep.distribution.uniform((-2, -2, 0), (2, 2, 0)),
        scale=rep.distribution.uniform((0.5, 0.5, 0.5), (1.5, 1.5, 1.5)),
        semantics=[rep.semantics.SemanticLabel("obstacle", 2)],
        count=10
    )
    
    # Set up camera and randomizers
    setup_camera_data_stream()
    randomize_lighting()
    randomize_materials()
    
    # Define data capture configuration
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir="./synthetic_datasets/humanoid_perception",
        rgb=True,
        depth=True,
        instance_segmentation=True
    )
    writer.attach([rep.RenderProduct("/Replicator/RenderProduct_0")])
    
    # Run the generation
    rep.orchestrator.run_until_complete(num_frames=5000)


# Alternative approach with more advanced domain randomization
def create_advanced_domain_randomization():
    """Create more sophisticated domain randomization"""
    
    # Randomize environment
    with rep.trigger.on_frame():
        # Randomize lighting
        lights = rep.get.prims_from_path("/World/Lights")
        with lights:
            rep.modify.light(
                position=rep.distribution.uniform((-50, -50, 100), (50, 50, 200)),
                intensity=rep.distribution.uniform(100, 1000),
                color=rep.distribution.uniform((0.7, 0.7, 0.7), (1.0, 1.0, 1.0))
            )
        
        # Randomize textures on ground
        ground = rep.get.prims_from_path("/World/Ground")
        with ground:
            rep.randomizer.material(
                inputs={
                    "inputs:diffuse_texture": rep.utils.get_usd_shade_mats_from_directory(
                        "Omni/Kit/Albedo/Textures", num=200
                    )
                }
            )
        
        # Randomize object positions and appearances
        objects = rep.get.prims_from_path("/World/Objects")
        with objects:
            rep.randomizer.position(
                position=rep.distribution.uniform((-4, -4, 0), (4, 4, 0))
            )
            rep.randomizer.rotation(
                rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 3.14))
            )
            rep.randomizer.material(
                inputs={
                    "inputs:diffuse_texture": rep.utils.get_usd_shade_mats_from_directory(
                        "Omni/Kit/Albedo/Textures/Materials", num=150
                    )
                }
            )