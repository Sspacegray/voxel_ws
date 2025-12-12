<template>
  <div class="waypoint-panel">
    <div class="panel-header">
      <h3>路点编辑器</h3>
      <div class="header-actions">
        <el-button size="small" circle @click="$emit('close')">
          <el-icon><Close /></el-icon>
        </el-button>
      </div>
    </div>

    <!-- 工具栏 -->
    <div class="tools-row">
      <el-button-group>
        <el-button 
          :type="isAdding ? 'primary' : 'default'" 
          size="small"
          @click="toggleAdding"
        >
          <el-icon><Plus /></el-icon> 添加
        </el-button>
        <el-button size="small" @click="captureCurrentPose">
          <el-icon><Aim /></el-icon> 捕获
        </el-button>
      </el-button-group>
      
      <el-dropdown trigger="click" @command="handleFileCommand">
        <el-button size="small">
          文件 <el-icon class="el-icon--right"><ArrowDown /></el-icon>
        </el-button>
        <template #dropdown>
          <el-dropdown-menu>
            <el-dropdown-item command="import_csv">导入 CSV</el-dropdown-item>
            <el-dropdown-item command="import_yaml">导入 YAML</el-dropdown-item>
            <el-dropdown-item command="export_csv" divided>导出 CSV</el-dropdown-item>
            <el-dropdown-item command="export_yaml">导出 YAML</el-dropdown-item>
          </el-dropdown-menu>
        </template>
      </el-dropdown>
    </div>

    <!-- 路点列表 -->
    <div class="list-container">
      <el-empty v-if="waypoints.length === 0" description="暂无路点" :image-size="60"></el-empty>
      
      <div 
        v-else 
        class="waypoint-item" 
        v-for="(wp, index) in waypoints" 
        :key="wp.id"
        :class="{ 'active': wp.id === activeWaypointId }"
        @click="selectWaypoint(wp.id)"
      >
        <div class="wp-index">{{ index + 1 }}</div>
        <div class="wp-info">
          <div class="wp-coords">
            X: {{ wp.x.toFixed(2) }} Y: {{ wp.y.toFixed(2) }}
          </div>
          <div class="wp-yaw">
            Yaw: {{ (wp.yaw * 180 / Math.PI).toFixed(1) }}°
          </div>
        </div>
        <div class="wp-actions">
           <el-button size="small" text circle type="danger" @click.stop="removeWaypoint(index)">
             <el-icon><Delete /></el-icon>
           </el-button>
        </div>
      </div>
    </div>

    <!-- 底部执行栏 -->
    <div class="execution-bar">
      <div class="loop-switch">
        <el-switch v-model="isLooping" size="small" active-text="循环" />
      </div>
      <el-button type="success" size="default" class="start-btn" @click="startNavigation">
        <el-icon><VideoPlay /></el-icon> 开始巡航
      </el-button>
    </div>
  </div>
</template>

<script>
import { ref, defineComponent } from 'vue'
import { 
  Plus, Aim, ArrowDown, Delete, VideoPlay, Close
} from '@element-plus/icons-vue'
import { ElMessage } from 'element-plus'

export default defineComponent({
  name: 'WaypointPanel',
  components: {
    Plus, Aim, ArrowDown, Delete, VideoPlay, Close
  },
  emits: ['close', 'tool-change'],
  setup(props, { emit }) {
    const waypoints = ref([])
    const isAdding = ref(false)
    const activeWaypointId = ref(null)
    const isLooping = ref(false)
    let idCounter = 0

    // 工具切换
    const toggleAdding = () => {
      isAdding.value = !isAdding.value
      // 通知 3D 场景切换到添加路点工具
      emit('tool-change', isAdding.value ? 'add_waypoint' : null)
    }

    // 捕获当前位置 (Mock)
    const captureCurrentPose = () => {
      // 实际应从 store 获取 robotPose
      const mockPose = { x: Math.random() * 10 - 5, y: Math.random() * 10 - 5, yaw: 0 }
      addWaypoint(mockPose)
      ElMessage.success('已捕获当前位置')
    }

    const addWaypoint = (pose) => {
      waypoints.value.push({
        id: ++idCounter,
        ...pose
      })
    }
    
    const removeWaypoint = (index) => {
      waypoints.value.splice(index, 1)
    }

    const selectWaypoint = (id) => {
      activeWaypointId.value = id
      // TODO: 通知 3D 场景高亮该点
    }

    const handleFileCommand = (cmd) => {
      ElMessage.info(`执行文件操作: ${cmd}`)
      // TODO: 实现具体的文件读写逻辑
    }

    const startNavigation = () => {
      if (waypoints.value.length === 0) {
        ElMessage.warning('请先添加路点')
        return
      }
      ElMessage.success(`开始执行 ${waypoints.value.length} 个路点的巡航任务`)
      // TODO: 发布导航目标队列
    }

    return {
      waypoints,
      isAdding,
      activeWaypointId,
      isLooping,
      toggleAdding,
      captureCurrentPose,
      removeWaypoint,
      selectWaypoint,
      handleFileCommand,
      startNavigation
    }
  }
})
</script>

<style scoped>
.waypoint-panel {
  width: 300px;
  max-height: 60vh;
  display: flex;
  flex-direction: column;
  background: rgba(15, 23, 42, 0.85); /* Dashboard BG */
  backdrop-filter: blur(12px);
  border: 1px solid rgba(6, 182, 212, 0.2);
  box-shadow: 0 0 15px rgba(6, 182, 212, 0.15);
  border-radius: 4px;
  color: #fff;
  font-family: 'Outfit', sans-serif;
  overflow: hidden;
  pointer-events: auto;
}

.panel-header {
  padding: 12px 16px;
  display: flex;
  justify-content: space-between;
  align-items: center;
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  background: rgba(0, 0, 0, 0.2);
}

.title {
  display: flex;
  align-items: center;
  gap: 8px;
  font-weight: 700;
  color: #06b6d4;
  font-size: 14px;
}

.toolbar {
  padding: 12px;
  display: flex;
  gap: 8px;
  border-bottom: 1px solid rgba(148, 163, 184, 0.1);
}

.tool-btn {
  flex: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
  padding: 8px;
  border: 1px solid rgba(148, 163, 184, 0.2);
  background: rgba(30, 41, 59, 0.5);
  color: #e2e8f0;
  border-radius: 2px;
  cursor: pointer;
  transition: all 0.2s;
  font-size: 12px;
  font-weight: 600;
}

.tool-btn:hover {
  background: rgba(6, 182, 212, 0.2);
  border-color: #06b6d4;
  color: #fff;
}

.tool-btn.active {
  background: #06b6d4;
  color: #0f172a;
  border-color: #06b6d4;
}

.waypoint-list {
  flex: 1;
  overflow-y: auto;
  padding: 8px;
  min-height: 150px;
}

.empty-tip {
  text-align: center;
  color: #64748b;
  padding: 24px;
  font-size: 12px;
}

.waypoint-item {
  display: flex;
  align-items: center;
  padding: 8px;
  margin-bottom: 4px;
  background: rgba(255, 255, 255, 0.03);
  border-radius: 2px;
  border-left: 2px solid transparent;
}

.waypoint-item:hover {
  background: rgba(255, 255, 255, 0.06);
}

.waypoint-item.active {
  border-left-color: #10b981;
  background: rgba(16, 185, 129, 0.1);
}

.index-badge {
  background: #334155;
  color: #94a3b8;
  width: 20px;
  height: 20px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-size: 10px;
  margin-right: 8px;
}

.waypoint-info {
  flex: 1;
  display: flex;
  flex-direction: column;
  font-size: 11px;
}

.coord-text {
  color: #cbd5e1;
  font-family: monospace;
}

.delete-btn {
  background: transparent;
  border: none;
  color: #64748b;
  cursor: pointer;
  padding: 4px;
}

.delete-btn:hover {
  color: #ef4444;
}

.file-actions {
  display: flex;
  gap: 1px;
  background: rgba(0, 0, 0, 0.2);
}

.file-action-btn {
  flex: 1;
  background: transparent;
  border: none;
  color: #94a3b8;
  padding: 8px;
  font-size: 11px;
  cursor: pointer;
  border-top: 1px solid rgba(148, 163, 184, 0.1);
}

.file-action-btn:hover {
  color: #06b6d4;
  background: rgba(255, 255, 255, 0.02);
}

.execution-controls {
  padding: 12px;
  background: rgba(0, 0, 0, 0.2);
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.loop-switch {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 12px;
  color: #cbd5e1;
  cursor: pointer;
}

.start-btn {
  width: 100%;
  padding: 10px;
  background: rgba(16, 185, 129, 0.2);
  border: 1px solid #10b981;
  color: #10b981;
  border-radius: 2px;
  font-weight: 700;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  transition: all 0.2s;
}

.start-btn:hover {
  background: #10b981;
  color: #000;
}
</style>
