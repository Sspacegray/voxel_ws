<template>
  <div class="map-manager-panel dashboard-panel">
    <div class="panel-header">
      <el-icon><MapLocation /></el-icon>
      <h3>MAP MANAGER</h3>
    </div>
    
    <div class="panel-content">
      <!-- Section 1: Map List & Load -->
      <div class="section-title">SAVED MAPS</div>
      <div class="map-list-container">
        <ul class="map-list">
          <li 
            v-for="map in maps" 
            :key="map.name" 
            :class="{ active: selectedMap === map.name }"
            @click="selectedMap = map.name"
          >
            <el-icon><Document /></el-icon>
            <span class="map-name">{{ map.name }}</span>
            <span class="map-date">{{ map.date }}</span>
          </li>
        </ul>
      </div>
      
      <div class="action-row">
        <button class="action-btn primary" @click="loadSelectedMap" :disabled="!selectedMap">
          LOAD MAP
        </button>
        <button class="action-btn danger" @click="deleteSelectedMap" :disabled="!selectedMap">
          DELETE
        </button>
      </div>

      <div class="divider"></div>

      <!-- Section 2: SLAM Mapping -->
      <div class="section-title">MAPPING</div>
      
      <div v-if="!isMapping" class="mapping-start-section">
         <div class="input-group">
           <label>NEW MAP NAME</label>
           <input v-model="newMapName" placeholder="Enter map name..." />
         </div>
         <button class="action-btn success full-width" @click="startMapping">
           <el-icon><VideoPlay /></el-icon>
           START MAPPING (SLAM)
         </button>
      </div>
      
      <div v-else class="mapping-active-section">
        <div class="recording-indicator">
          <div class="rec-dot"></div>
          RECORDING...
        </div>
        <button class="action-btn primary full-width" @click="saveMap">
          <el-icon><WhichAreVery /></el-icon>
          SAVE MAP
        </button>
        <button class="action-btn warning full-width" @click="cancelMapping">
          CANCEL
        </button>
      </div>
    </div>
  </div>
</template>

<script>
import { ref, onMounted } from 'vue'
import { MapLocation, Document, VideoPlay, Camera as WhichAreVery, Refresh } from '@element-plus/icons-vue'
import { ElMessage } from 'element-plus'

export default {
  name: 'MapManagerPanel',
  components: { MapLocation, Document, VideoPlay, WhichAreVery, Refresh },
  setup() {
    const maps = ref([])
    const selectedMap = ref(null)
    const isMapping = ref(false)
    const newMapName = ref('')
    const loading = ref(false)
    
    const fetchMaps = async () => {
      loading.value = true
      try {
        const res = await fetch('http://localhost:8000/api/v1/maps')
        if (res.ok) {
           maps.value = await res.json()
        }
      } catch (e) {
        console.error("Failed to fetch maps", e)
      } finally {
        loading.value = false
      }
    }

    onMounted(() => {
      fetchMaps()
    })
    
    const loadSelectedMap = async () => {
      if(!selectedMap.value) return
      try {
        // Post current map name to backend
        const res = await fetch(`http://localhost:8000/api/v1/maps/load?map_name=${selectedMap.value}`, {
            method: 'POST'
        })
        if (res.ok) {
            ElMessage.success(`Map ${selectedMap.value} Loaded`)
        }
      } catch (e) {
        ElMessage.error("Failed to load map")
      }
    }
    
    const deleteSelectedMap = () => {
      ElMessage.warning("Delete not implemented in Pro version yet")
    }
    
    const startMapping = () => {
      isMapping.value = true
      // Call ROS service to start SLAM
      ElMessage.info("SLAM Started")
    }
    
    const saveMap = () => {
      console.log('Saving map as:', newMapName.value)
      isMapping.value = false
      // Call map_server saver
      ElMessage.success("Map Saved")
      fetchMaps() // Refresh list
    }
    
    const cancelMapping = () => {
      isMapping.value = false
    }
    
    return {
      maps,
      selectedMap,
      isMapping,
      newMapName,
      loading,
      fetchMaps,
      loadSelectedMap,
      deleteSelectedMap,
      startMapping,
      saveMap,
      cancelMapping
    }
  }
}
</script>

<style scoped>
.map-manager-panel {
  width: 320px;
  padding: 16px;
  display: flex;
  flex-direction: column;
  gap: 16px;
  font-family: 'Outfit', sans-serif;
}

.panel-header {
  display: flex;
  align-items: center;
  gap: 10px;
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  padding-bottom: 12px;
  color: #06b6d4;
  font-size: 14px;
  font-weight: 700;
  letter-spacing: 1px;
}

.section-title {
  font-size: 11px;
  font-weight: 600;
  color: #64748b;
  margin-bottom: 8px;
  letter-spacing: 0.5px;
}

.map-list-container {
  background: rgba(0, 0, 0, 0.2);
  border: 1px solid rgba(148, 163, 184, 0.1);
  border-radius: 4px;
  max-height: 200px;
  overflow-y: auto;
  margin-bottom: 12px;
}

.map-list {
  list-style: none;
  padding: 4px;
}

.map-list li {
  display: flex;
  align-items: center;
  gap: 8px;
  padding: 8px 12px;
  border-radius: 2px;
  cursor: pointer;
  color: #94a3b8;
  font-size: 13px;
  transition: all 0.2s;
}

.map-list li:hover {
  background: rgba(255, 255, 255, 0.05);
  color: #fff;
}

.map-list li.active {
  background: rgba(6, 182, 212, 0.15);
  color: #06b6d4;
  border-left: 2px solid #06b6d4;
}

.map-date {
  margin-left: auto;
  font-size: 10px;
  opacity: 0.6;
}

.action-row {
  display: flex;
  gap: 8px;
}

.action-btn {
  flex: 1;
  padding: 8px;
  border: none;
  border-radius: 2px;
  font-weight: 600;
  font-size: 12px;
  cursor: pointer;
  transition: all 0.2s;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 6px;
}

.action-btn.primary {
  background: rgba(6, 182, 212, 0.2);
  border: 1px solid #06b6d4;
  color: #06b6d4;
}

.action-btn.primary:hover {
  background: #06b6d4;
  color: #000;
}

.action-btn.success {
  background: rgba(16, 185, 129, 0.2);
  border: 1px solid #10b981;
  color: #10b981;
}

.action-btn.success:hover {
  background: #10b981;
  color: #000;
}

.action-btn.danger {
  background: rgba(239, 68, 68, 0.1);
  border: 1px solid rgba(239, 68, 68, 0.5);
  color: #ef4444;
}

.action-btn.danger:hover {
  background: #ef4444;
  color: #fff;
}

.action-btn.warning {
  background: rgba(245, 158, 11, 0.2);
  border: 1px solid #f59e0b;
  color: #f59e0b;
}

.action-btn:disabled {
  opacity: 0.5;
  cursor: not-allowed;
  filter: grayscale(100%);
}

.full-width {
  width: 100%;
}

.divider {
  height: 1px;
  background: rgba(148, 163, 184, 0.1);
  margin: 8px 0;
}

.input-group {
  display: flex;
  flex-direction: column;
  gap: 6px;
  margin-bottom: 12px;
}

.input-group label {
  font-size: 10px;
  color: #64748b;
  font-weight: 600;
}

.input-group input {
  background: rgba(15, 23, 42, 0.6);
  border: 1px solid #334155;
  padding: 8px;
  border-radius: 2px;
  color: white;
  font-family: inherit;
  font-size: 13px;
}

.input-group input:focus {
  outline: none;
  border-color: #06b6d4;
}

.recording-indicator {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  color: #ef4444;
  font-weight: 700;
  font-size: 12px;
  margin-bottom: 12px;
  animation: pulse 2s infinite;
}

.rec-dot {
  width: 8px;
  height: 8px;
  background: #ef4444;
  border-radius: 50%;
}

@keyframes pulse {
  0% { opacity: 1; }
  50% { opacity: 0.5; }
  100% { opacity: 1; }
}

.mapping-active-section {
  display: flex;
  flex-direction: column;
  gap: 8px;
}
</style>
