import {
    Box, Typography, Button, Stack, TextField
} from "@mui/material";
import { useState } from "react";
import ConfigBlockEditor from "./ConfigBlockEditor";
import { ConfigBlock, FullConfig } from "./types";

export default function ConfigBuilder({
    onSave
}: { onSave: (cfg: FullConfig) => void }) {
    const [name, setName] = useState("my_config");
    const [blocks, setBlocks] = useState<ConfigBlock[]>([]);

    const addNode = () => {
        setBlocks([...blocks, {
            type: "node",
            name: "",
            package: "",
            executable: "",
            parameters: {},
            remaps: {}
        }]);
    };

    const addLaunch = () => {
        setBlocks([...blocks, {
            type: "launch",
            package: "",
            path: "",
            arguments: {}
        }]);
    };

    const updateBlock = (i: number, updated: ConfigBlock) => {
        setBlocks(blocks.map((b, idx) => idx === i ? updated : b));
    };

    const deleteBlock = (i: number) => {
        setBlocks(blocks.filter((_, idx) => idx !== i));
    };

    return (
        <Box>
            <Typography variant="h6" gutterBottom>Build Configuration</Typography>
            <TextField fullWidth label="Config Name" value={name} onChange={(e) => setName(e.target.value)} sx={{ mb: 2 }} />

            <Stack spacing={2}>
                {blocks.map((block, idx) => (
                    <ConfigBlockEditor
                        key={idx}
                        block={block}
                        onChange={(b) => updateBlock(idx, b)}
                        onDelete={() => deleteBlock(idx)}
                    />
                ))}
            </Stack>

            <Box mt={2} display="flex" gap={2}>
                <Button variant="outlined" onClick={addNode}>+ Add Node</Button>
                <Button variant="outlined" onClick={addLaunch}>+ Add Launch File</Button>
                <Box flexGrow={1} />
                <Button variant="contained" onClick={() => onSave({ name, blocks })}>Save Config</Button>
            </Box>
        </Box>
    );
}
