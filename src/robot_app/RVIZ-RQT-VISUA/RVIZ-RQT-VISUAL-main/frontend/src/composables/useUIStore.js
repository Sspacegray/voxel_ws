import { defineStore } from 'pinia'
import { ref } from 'vue'

export const useUIStore = defineStore('ui', () => {
    // Layers State
    const layers = ref({
        robot: true,
        laser: true,
        map: true,
        points: true,
        tf: false,
        grid: true
    })

    // Speed Limit State (Global)
    const speedLimits = ref({
        linear: 0.5,
        angular: 1.0
    })

    // Actions
    function toggleLayer(layerId) {
        if (layerId in layers.value) {
            layers.value[layerId] = !layers.value[layerId]
        }
    }

    function setSpeedLimit(linear, angular) {
        speedLimits.value.linear = linear
        speedLimits.value.angular = angular
    }

    return {
        layers,
        speedLimits,
        toggleLayer,
        setSpeedLimit
    }
})
