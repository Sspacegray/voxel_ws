# bt_ai_backend（本地任务生成服务 / Stub）

这个目录提供一个**零依赖**的本地 HTTP 服务示例，用于配合 `tools/bt_visual_editor` 的 “AI任务” 按钮。

它不是强绑定某个大模型：你可以把这里替换成任意后端（本地 LLM、OpenAI、公司内部服务等），只要返回同样的 JSON 格式即可。

## 运行

在工作区根目录执行：

```bash
python3 tools/bt_ai_backend/server.py --host 127.0.0.1 --port 8787
```

然后在浏览器里的编辑器点击 “AI任务”，后端 URL 填：

`http://127.0.0.1:8787/generate`

## 接口约定

### 请求

`POST /generate`

```json
{
  "prompt": "自然语言任务描述",
  "template_id": "nav2_patrol_through_poses"
}
```

### 响应

```json
{
  "mission": { "..." : "任务计划 JSON" },
  "bt_xml": "<root ...>...</root>",
  "waypoints_yaml": "frame_id: map\nwaypoints: ...\n"
}
```

