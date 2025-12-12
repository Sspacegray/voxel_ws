<template>
  <div class="speed-control-panel dashboard-panel">
    <div class="panel-header">
      <el-icon><Odometer /></el-icon>
      <h3>SPEED LIMITS</h3>
    </div>

    <div class="speed-slider-group">
      <div class="slider-header">
        <span class="label">MAX LINEAR</span>
        <span class="value">{{ maxLinear.toFixed(1) }} m/s</span>
      </div>
      <el-slider 
        v-model="maxLinear" 
        :min="0.1" 
        :max="2.0" 
        :step="0.1" 
        :show-tooltip="false"
        class="custom-slider"
      />
    </div>

    <div class="speed-slider-group">
      <div class="slider-header">
        <span class="label">MAX ANGULAR</span>
        <span class="value">{{ maxAngular.toFixed(1) }} rad/s</span>
      </div>
      <el-slider 
        v-model="maxAngular" 
        :min="0.1" 
        :max="3.0" 
        :step="0.1" 
        :show-tooltip="false"
        class="custom-slider"
      />
    </div>
  </div>
</template>

<script>
import { ref, watch } from 'vue'
import { Odometer } from '@element-plus/icons-vue'

export default {
  name: 'SpeedControlPanel',
  components: { Odometer },
  emits: ['speed-change'],
  setup(props, { emit }) {
    const maxLinear = ref(0.5)
    const maxAngular = ref(1.0)

    watch([maxLinear, maxAngular], ([lin, ang]) => {
      emit('speed-change', { linear: lin, angular: ang })
    })

    return {
      maxLinear,
      maxAngular
    }
  }
}
</script>

<style scoped>
.speed-control-panel {
  width: 240px;
  padding: 16px;
  display: flex;
  flex-direction: column;
  gap: 16px;
}

.panel-header {
  display: flex;
  align-items: center;
  gap: 10px;
  color: #06b6d4;
  font-size: 14px;
  font-weight: 700;
  border-bottom: 1px solid rgba(148, 163, 184, 0.2);
  padding-bottom: 12px;
}

.speed-slider-group {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.slider-header {
  display: flex;
  justify-content: space-between;
  font-size: 11px;
  font-weight: 600;
}

.label {
  color: #64748b;
}

.value {
  color: #fff;
  font-family: monospace;
}

/* Custom Slider Styling for Sci-Fi Look */
:deep(.el-slider__runway) {
  background-color: rgba(148, 163, 184, 0.2);
  height: 4px;
}

:deep(.el-slider__bar) {
  background-color: #06b6d4;
  height: 4px;
  box-shadow: 0 0 10px rgba(6, 182, 212, 0.5);
}

:deep(.el-slider__button) {
  width: 12px;
  height: 12px;
  border: 2px solid #06b6d4;
  background-color: #0f172a;
}
</style>
