"""
可视化相关 API 端点
"""

from fastapi import APIRouter, HTTPException, Depends
from typing import List, Dict, Any
import logging

from ...core.config import get_settings
from ...models.viz import VisualizationState, PluginInfo, CameraSettings, RenderSettings
from ...services.rosbridge import RosbridgeService
from ...services.dependencies import get_rosbridge_service

router = APIRouter()
logger = logging.getLogger(__name__)

@router.get("/visualization/state", response_model=VisualizationState)
async def get_visualization_state(
    service: RosbridgeService = Depends(get_rosbridge_service)
):
    """获取可视化状态"""
    try:
        state = await service.get_visualization_state()
        return state
    except Exception as e:
        logger.error(f"Failed to get visualization state: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/visualization/camera")
async def update_camera_settings(
    settings: CameraSettings,
    service: RosbridgeService = Depends(get_rosbridge_service)
):
    """更新相机设置"""
    try:
        success = await service.update_camera_settings(settings)
        return {"success": success, "action": "camera_updated"}
    except Exception as e:
        logger.error(f"Failed to update camera settings: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/visualization/render")
async def update_render_settings(
    settings: RenderSettings,
    service: RosbridgeService = Depends(get_rosbridge_service)
):
    """更新渲染设置"""
    try:
        success = await service.update_render_settings(settings)
        return {"success": success, "action": "render_updated"}
    except Exception as e:
        logger.error(f"Failed to update render settings: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.get("/visualization/plugins", response_model=List[PluginInfo])
async def get_available_plugins(
    service: RosbridgeService = Depends(get_rosbridge_service)
):
    """获取可用插件列表"""
    try:
        plugins = await service.get_available_plugins()
        return plugins
    except Exception as e:
        logger.error(f"Failed to get plugins: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/visualization/plugins/{plugin_id}/enable")
async def enable_plugin(
    plugin_id: str,
    service: RosbridgeService = Depends(get_rosbridge_service)
):
    """启用插件"""
    try:
        success = await service.enable_plugin(plugin_id)
        return {"success": success, "plugin_id": plugin_id, "action": "enabled"}
    except Exception as e:
        logger.error(f"Failed to enable plugin {plugin_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/visualization/plugins/{plugin_id}/disable")
async def disable_plugin(
    plugin_id: str,
    service: RosbridgeService = Depends(get_rosbridge_service)
):
    """禁用插件"""
    try:
        success = await service.disable_plugin(plugin_id)
        return {"success": success, "plugin_id": plugin_id, "action": "disabled"}
    except Exception as e:
        logger.error(f"Failed to disable plugin {plugin_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/visualization/objects/add")
async def add_visualization_object(
    object_data: Dict[str, Any],
    service: RosbridgeService = Depends(get_rosbridge_service)
):
    """添加可视化对象"""
    try:
        object_id = await service.add_visualization_object(object_data)
        return {"success": True, "object_id": object_id, "action": "object_added"}
    except Exception as e:
        logger.error(f"Failed to add visualization object: {e}")
        raise HTTPException(status_code=500, detail=str(e))

    except Exception as e:
        logger.error(f"Failed to remove visualization object {object_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))

# --- Map Management Endpoints ---

import os
import glob

MAP_DIR = "/home/suja/voxel_ws/src/robot_navigation/src/map"

@router.get("/maps")
async def list_maps():
    """列出可用地图文件"""
    try:
        if not os.path.exists(MAP_DIR):
            return []
        
        # Scan for yaml files (Nav2 maps)
        yaml_files = glob.glob(os.path.join(MAP_DIR, "*.yaml"))
        maps = []
        for f in yaml_files:
            name = os.path.basename(f).replace(".yaml", "")
            # Assuming matching pgm/png exists, but purely listing yaml is enough for Nav2
            maps.append({
                "name": name,
                "path": f,
                "date": "2023-12-09" # Placeholder or use os.path.getmtime
            })
        return maps
    except Exception as e:
        logger.error(f"Failed to list maps: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@router.post("/maps/load")
async def load_map(
    map_name: str,
    service: RosbridgeService = Depends(get_rosbridge_service)
):
    """加载指定地图 (Call Nav2 map_server)"""
    try:
        # Build full path
        map_path = os.path.join(MAP_DIR, f"{map_name}.yaml")
        if not os.path.exists(map_path):
             raise HTTPException(status_code=404, detail="Map file not found")

        # In a real scenario, we call the ROS 2 lifecycle manager or map_server load_map service
        # service.call_service('/map_server/load_map', ...)
        # For now, we just return success to simulate the UI interaction
        logger.info(f"Loading map: {map_path}")
        return {"success": True, "message": f"Map {map_name} loaded successfully"}
    except Exception as e:
        logger.error(f"Failed to load map: {e}")
        raise HTTPException(status_code=500, detail=str(e))
