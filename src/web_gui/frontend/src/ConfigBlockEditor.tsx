// src/ConfigBlockEditor.tsx
import { NodeConfig, LaunchFileConfig } from "./types";
import NodeEditor from "./components/NodeEditor";          // <- separate if desired
import LaunchEditor from "./components/LaunchEditor";      // <- separate if desired

export default function ConfigBlockEditor({
    block,
    onChange,
    onDelete
}: {
    block: NodeConfig | LaunchFileConfig;
    onChange: (updated: typeof block) => void;
    onDelete: () => void;
}) {
    if (block.type === "node") {
        return <NodeEditor block={block} onChange={onChange} onDelete={onDelete} />;
    } else {
        return <LaunchEditor block={block} onChange={onChange} onDelete={onDelete} />;
    }
}
