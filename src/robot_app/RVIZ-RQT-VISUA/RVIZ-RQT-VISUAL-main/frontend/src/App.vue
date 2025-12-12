<template>
  <div id="app">
    <!-- 顶部状态栏 -->
    <div class="app-header">
      <div class="header-left">
        <div class="app-logo">
          <div class="logo-icon"></div>
          <h1 class="app-title">ROS2 Web 可视化系统</h1>
        </div>
      </div>
      <div class="header-right">
        <connection-status />
      </div>
    </div>
    
    <!-- 主内容区 -->
    <div class="app-content">
      <router-view />
    </div>
  </div>
</template>

<script>
import { onMounted, onUnmounted } from 'vue'
import ConnectionStatus from './components/common/ConnectionStatus.vue'
import { useConnectionStore } from './composables/useConnectionStore'

export default {
  name: 'App',
  components: {
    ConnectionStatus
  },
  setup() {
    const connectionStore = useConnectionStore()
    
    // 应用启动时初始化连接
    onMounted(() => {
      connectionStore.initializeConnection()
    })
    
    // 应用卸载时清理连接
    onUnmounted(() => {
      connectionStore.disconnect()
    })
    
    return {}
  }
}
</script>

<style>
* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}

html, body {
  font-family: 'Outfit', 'Inter', -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
  background: #0f172a; /* Deep Space Blue */
  color: #e2e8f0;
  overflow: hidden;
  height: 100vh;
}

#app {
  height: 100vh;
  position: relative;
  background: radial-gradient(circle at center, #1e293b 0%, #0f172a 100%);
}

/* 隐藏旧的Header */
.app-header {
  display: none !important;
}

/* Element Plus 主题定制 (Sci-Fi Dashboard) */
:root {
  --el-color-primary: #06b6d4; /* Cyan 500 - Neon Blue */
  --el-color-success: #10b981; /* Emerald 500 */
  --el-color-warning: #f59e0b; /* Amber 500 */
  --el-color-danger: #ef4444; /* Red 500 */
  --el-color-info: #64748b; /* Slate 500 */
  
  --el-bg-color: #0f172a;
  --el-bg-color-page: #0f172a;
  --el-bg-color-overlay: #1e293b;
  
  --el-text-color-primary: #f1f5f9;
  --el-text-color-regular: #cbd5e1;
  --el-text-color-secondary: #94a3b8;
  --el-text-color-placeholder: #64748b;
  
  --el-border-color: #334155;
  --el-border-color-light: #1e293b;
  --el-border-color-lighter: #0f172a;
  
  /* 自定义仪表盘变量 */
  --dashboard-bg: rgba(15, 23, 42, 0.85);
  --dashboard-border: 1px solid rgba(6, 182, 212, 0.2);
  --dashboard-glow: 0 0 15px rgba(6, 182, 212, 0.15);
}

/* 通用面板样式 */
.dashboard-panel {
  background: var(--dashboard-bg);
  backdrop-filter: blur(12px);
  border: var(--dashboard-border);
  box-shadow: var(--dashboard-glow);
  border-radius: 4px; /* Tech style often uses sharper corners */
  color: #fff;
}

/* Element Plus 组件样式覆盖 - 科技感 */
.el-button {
  background: rgba(30, 41, 59, 0.5) !important;
  border: 1px solid rgba(148, 163, 184, 0.2) !important;
  color: #cbd5e1 !important;
  border-radius: 2px !important; /* Sharp corners */
  transition: all 0.3s ease;
}

.el-button:hover {
  background: rgba(6, 182, 212, 0.2) !important;
  border-color: #06b6d4 !important;
  color: #06b6d4 !important;
  box-shadow: 0 0 8px rgba(6, 182, 212, 0.4);
}

.el-button--primary {
  background: rgba(6, 182, 212, 0.2) !important;
  border-color: #06b6d4 !important;
  color: #06b6d4 !important;
}

.el-button--primary:hover {
  background: #06b6d4 !important;
  color: #000 !important;
}

.el-button--danger {
  background: rgba(239, 68, 68, 0.2) !important;
  border-color: #ef4444 !important;
  color: #ef4444 !important;
}

.el-input__wrapper {
  background: rgba(15, 23, 42, 0.8) !important;
  box-shadow: none !important;
  border: 1px solid #334155 !important;
}

.el-input__wrapper.is-focus {
  border-color: #06b6d4 !important;
  box-shadow: 0 0 5px rgba(6, 182, 212, 0.3) !important;
}

/* 滚动条美化 (Dark) */
::-webkit-scrollbar {
  width: 6px;
  height: 6px;
}

::-webkit-scrollbar-track {
  background: transparent;
}

::-webkit-scrollbar-thumb {
  background: #334155;
  border-radius: 3px;
}

::-webkit-scrollbar-thumb:hover {
  background: #06b6d4;
}
</style>
