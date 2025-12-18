<template>
  <div class="system-info">
    <!-- 系统状态 -->
    <div class="status-overview">
      <div class="status-grid">
        <div class="status-item">
          <div class="status-icon">🤖</div>
          <div class="status-content">
            <div class="status-value">{{ systemStatus.activeNodes }}</div>
            <div class="status-label">活跃节点</div>
          </div>
        </div>
        
        <div class="status-item">
          <div class="status-icon">📡</div>
          <div class="status-content">
            <div class="status-value">{{ systemStatus.activeTopics }}</div>
            <div class="status-label">活跃主题</div>
          </div>
        </div>
        
        <div class="status-item">
          <div class="status-icon">🔗</div>
          <div class="status-content">
            <div class="status-value">{{ systemStatus.connections }}</div>
            <div class="status-label">连接数</div>
          </div>
        </div>
        
        <div class="status-item">
          <div class="status-icon">⚡</div>
          <div class="status-content">
            <div class="status-value">{{ formatUptime(systemStatus.uptime) }}</div>
            <div class="status-label">运行时间</div>
          </div>
        </div>
      </div>
    </div>
    
    <!-- 性能指标 -->
    <el-divider />
    <div class="performance-metrics">
      <h4>性能指标</h4>
      <div class="metrics-grid">
        <div class="metric-item">
          <div class="metric-label">CPU 使用率</div>
          <el-progress 
            :percentage="systemStatus.cpuUsage" 
            :color="getProgressColor(systemStatus.cpuUsage)"
            :stroke-width="8"
          />
        </div>
        
        <div class="metric-item">
          <div class="metric-label">内存使用率</div>
          <el-progress 
            :percentage="systemStatus.memoryUsage" 
            :color="getProgressColor(systemStatus.memoryUsage)"
            :stroke-width="8"
          />
        </div>
        
        <div class="metric-item">
          <div class="metric-label">消息频率</div>
          <div class="metric-value">{{ systemStatus.messageRate.toFixed(1) }} Hz</div>
        </div>
        
        <div class="metric-item">
          <div class="metric-label">网络延迟</div>
          <div class="metric-value">{{ systemStatus.networkLatency }} ms</div>
        </div>
      </div>
    </div>
    
    <!-- ROS 信息 -->
    <el-divider />
    <div class="ros-info">
      <h4>ROS 信息</h4>
      <el-descriptions :column="1" size="small">
        <el-descriptions-item label="ROS 版本">
          {{ rosInfo.version }}
        </el-descriptions-item>
        <el-descriptions-item label="域 ID">
          {{ rosInfo.domainId }}
        </el-descriptions-item>
        <el-descriptions-item label="中间件">
          {{ rosInfo.middleware }}
        </el-descriptions-item>
        <el-descriptions-item label="主机名">
          {{ rosInfo.hostname }}
        </el-descriptions-item>
      </el-descriptions>
    </div>
  </div>
</template>

<script>
import { ref, reactive, onMounted, onUnmounted } from 'vue'

export default {
  name: 'SystemInfo',
  setup() {
    // 系统状态
    const systemStatus = reactive({
      activeNodes: 5,
      activeTopics: 12,
      connections: 3,
      uptime: 0,
      cpuUsage: 45,
      memoryUsage: 62,
      messageRate: 125.6,
      networkLatency: 15
    })
    
    // ROS 信息
    const rosInfo = reactive({
      version: 'ROS2 Jazzy',
      domainId: 0,
      middleware: 'rmw_fastrtps_cpp',
      hostname: 'robot-pc'
    })
    
    let updateInterval = null
    
    // 格式化运行时间
    const formatUptime = (seconds) => {
      const hours = Math.floor(seconds / 3600)
      const minutes = Math.floor((seconds % 3600) / 60)
      const secs = seconds % 60
      
      if (hours > 0) {
        return `${hours}h ${minutes}m`
      } else if (minutes > 0) {
        return `${minutes}m ${secs}s`
      } else {
        return `${secs}s`
      }
    }
    
    // 获取进度条颜色
    const getProgressColor = (percentage) => {
      if (percentage < 50) return '#67C23A'
      if (percentage < 80) return '#E6A23C'
      return '#F56C6C'
    }
    
    // 更新系统状态
    const updateSystemStatus = () => {
      // 模拟数据更新
      systemStatus.uptime += 1
      systemStatus.cpuUsage = Math.max(0, Math.min(100, systemStatus.cpuUsage + (Math.random() - 0.5) * 10))
      systemStatus.memoryUsage = Math.max(0, Math.min(100, systemStatus.memoryUsage + (Math.random() - 0.5) * 5))
      systemStatus.messageRate = Math.max(0, systemStatus.messageRate + (Math.random() - 0.5) * 20)
      systemStatus.networkLatency = Math.max(1, Math.min(100, systemStatus.networkLatency + (Math.random() - 0.5) * 10))
      
      // 偶尔更新节点和主题数量
      if (Math.random() < 0.1) {
        systemStatus.activeNodes = Math.max(1, systemStatus.activeNodes + Math.floor((Math.random() - 0.5) * 3))
        systemStatus.activeTopics = Math.max(1, systemStatus.activeTopics + Math.floor((Math.random() - 0.5) * 5))
        systemStatus.connections = Math.max(0, systemStatus.connections + Math.floor((Math.random() - 0.5) * 2))
      }
    }
    
    // 刷新系统信息
    const refresh = async () => {
      try {
        // 模拟刷新延迟
        await new Promise(resolve => setTimeout(resolve, 300))
        
        // 重置一些值
        systemStatus.activeNodes = Math.floor(Math.random() * 10) + 3
        systemStatus.activeTopics = Math.floor(Math.random() * 20) + 8
        systemStatus.connections = Math.floor(Math.random() * 5) + 1
        
        ElMessage.success('系统信息已刷新')
      } catch (error) {
        ElMessage.error('刷新失败')
      }
    }
    
    // 获取系统数据（供父组件调用）
    const getSystemData = () => {
      return {
        status: { ...systemStatus },
        rosInfo: { ...rosInfo },
        timestamp: new Date().toISOString()
      }
    }
    
    onMounted(() => {
      // 每秒更新一次状态
      updateInterval = setInterval(updateSystemStatus, 1000)
      
      // 初始化运行时间
      systemStatus.uptime = Math.floor(Math.random() * 3600) + 300 // 5分钟到1小时
    })
    
    onUnmounted(() => {
      if (updateInterval) {
        clearInterval(updateInterval)
      }
    })
    
    return {
      systemStatus,
      rosInfo,
      formatUptime,
      getProgressColor,
      // 暴露方法供父组件调用
      refresh,
      getSystemData
    }
  }
}
</script>

<style scoped>
.system-info {
  height: 100%;
  padding: 10px;
  overflow-y: auto;
}

.system-info h4 {
  margin: 0 0 10px 0;
  font-size: 14px;
  color: #333;
}

.status-overview {
  margin-bottom: 15px;
}

.status-grid {
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 10px;
}

.status-item {
  display: flex;
  align-items: center;
  padding: 8px;
  background: #f9f9f9;
  border-radius: 6px;
  border: 1px solid #e6e6e6;
}

.status-icon {
  font-size: 20px;
  margin-right: 8px;
}

.status-content {
  flex: 1;
}

.status-value {
  font-size: 16px;
  font-weight: bold;
  color: #409eff;
  line-height: 1;
}

.status-label {
  font-size: 11px;
  color: #666;
  margin-top: 2px;
}

.performance-metrics {
  margin-bottom: 15px;
}

.metrics-grid {
  display: grid;
  grid-template-columns: 1fr;
  gap: 12px;
}

.metric-item {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.metric-label {
  font-size: 12px;
  color: #666;
  width: 70px;
  flex-shrink: 0;
}

.metric-value {
  font-size: 13px;
  font-weight: 500;
  color: #333;
  min-width: 60px;
  text-align: right;
}

.ros-info {
  margin-bottom: 15px;
}

.ros-info :deep(.el-descriptions-item__label) {
  font-size: 12px;
  width: 80px;
}

.ros-info :deep(.el-descriptions-item__content) {
  font-size: 12px;
}
</style>
