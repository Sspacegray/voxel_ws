import { shallowRef } from 'vue'
import * as THREE from 'three'

// 简单的Base64解码工具
function decodeBase64(base64) {
    const binaryString = window.atob(base64)
    const len = binaryString.length
    const bytes = new Uint8Array(len)
    for (let i = 0; i < len; i++) {
        bytes[i] = binaryString.charCodeAt(i)
    }
    return bytes.buffer
}

export function usePointCloud(scene) {
    const pointClouds = new Map() // topic -> THREE.Points
    const MATERIAL_SETTINGS = {
        size: 0.15,
        vertexColors: true,
        sizeAttenuation: false,
        alphaTest: 0.5
    }

    const updatePointCloud = (topic, message) => {
        // 移除旧的点云（如果是全量更新）
        // 注意：这里简单起见，每次都重新创建几何体。生产环境应使用 BufferGeometry 更新
        if (pointClouds.has(topic)) {
            scene.remove(pointClouds.get(topic))
            pointClouds.delete(topic)
        }

        if (!message || !message.data) return

        const { width, height, point_step, row_step, fields, data, is_bigendian } = message
        // data 可能是 Base64 字符串
        const buffer = typeof data === 'string' ? decodeBase64(data) : data
        const dv = new DataView(buffer)
        const numPoints = width * height

        // 解析字段偏移量
        let xOffset = -1, yOffset = -1, zOffset = -1
        let rgbOffset = -1, intensityOffset = -1

        fields.forEach(f => {
            if (f.name === 'x') xOffset = f.offset
            if (f.name === 'y') yOffset = f.offset
            if (f.name === 'z') zOffset = f.offset
            if (f.name === 'rgb') rgbOffset = f.offset
            if (f.name === 'intensity') intensityOffset = f.offset
        })

        if (xOffset < 0 || yOffset < 0 || zOffset < 0) return

        const positions = []
        const colors = []

        for (let i = 0; i < numPoints; i++) {
            const pointStart = i * point_step

            // 读取坐标 (Float32, Little Endian)
            const x = dv.getFloat32(pointStart + xOffset, !is_bigendian)
            const y = dv.getFloat32(pointStart + yOffset, !is_bigendian)
            const z = dv.getFloat32(pointStart + zOffset, !is_bigendian)

            if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue

            positions.push(x, y, z)

            // 简单的颜色映射：根据高度 Z
            // 实际逻辑可更复杂 (Intensity/RGB)
            const zNorm = Math.min(Math.max((z + 1) / 3, 0), 1)
            colors.push(zNorm, 1.0 - zNorm, 0.5)
        }

        if (positions.length === 0) return

        const geometry = new THREE.BufferGeometry()
        geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3))
        geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3))

        const material = new THREE.PointsMaterial(MATERIAL_SETTINGS)
        const points = new THREE.Points(geometry, material)

        scene.add(points)
        pointClouds.set(topic, points)
    }

    const clearPointClouds = () => {
        pointClouds.forEach(p => scene.remove(p))
        pointClouds.clear()
    }

    return {
        updatePointCloud,
        clearPointClouds
    }
}
