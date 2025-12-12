<template>
  <transition name="slide-right">
    <div class="side-control-panel-container" v-if="isVisible">
      <!-- 这是一个容器，根据父组件传入的模式显示不同的子面板 -->
      
      <!-- 导航模式: 显示地图管理 -->
      <div v-if="mode === 'navigation' || mode === 'mapping'" class="panel-group">
        <MapManagerPanel />
      </div>

      <!-- 路点模式: 显示路点编辑器 (已有的 WaypointPanel 需适配 dashboard 样式) -->
      <div v-if="mode === 'waypoint'" class="panel-group">
        <WaypointPanel />
      </div>

      <!-- 手动/通用模式: 显示速度控制和手动摇杆 -->
      <div v-if="mode === 'manual' || mode === 'navigation' || mode === 'mapping'" class="panel-group bottom-group">
        <SpeedControlPanel @speed-change="onSpeedChange" />
      </div>
      
    </div>
  </transition>
</template>

<script>
import { computed } from 'vue'
import MapManagerPanel from './MapManagerPanel.vue'
import SpeedControlPanel from './SpeedControlPanel.vue'
import WaypointPanel from './WaypointPanel.vue'

export default {
  name: 'SideControlPanel',
  components: {
    MapManagerPanel,
    SpeedControlPanel,
    WaypointPanel
  },
  props: {
    mode: {
      type: String,
      default: 'monitor'
    }
  },
  emits: ['speed-change'],
  setup(props, { emit }) {
    const isVisible = computed(() => {
      return props.mode !== 'monitor'
    })

    const onSpeedChange = (speeds) => {
      emit('speed-change', speeds)
    }

    return {
      isVisible,
      onSpeedChange
    }
  }
}
</script>

<style scoped>
.side-control-panel-container {
  position: absolute;
  top: 80px;
  right: 20px;
  bottom: 20px;
  display: flex;
  flex-direction: column;
  gap: 16px;
  pointer-events: none; /* Container transparent to clicks */
  z-index: 90;
}

.panel-group {
  pointer-events: auto; /* Enable clicks on panels */
}

/* Slide Animation */
.slide-right-enter-active,
.slide-right-leave-active {
  transition: all 0.3s cubic-bezier(0.25, 0.8, 0.5, 1);
}

.slide-right-enter-from,
.slide-right-leave-to {
  opacity: 0;
  transform: translateX(50px);
}
</style>
