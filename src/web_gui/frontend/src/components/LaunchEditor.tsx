import { Box, Button, FormControl, IconButton, InputLabel, MenuItem, Select, Stack, TextField, Typography } from "@mui/material";
import DeleteIcon from "@mui/icons-material/Delete";
import { API_BASE, LaunchFileConfig } from "../types";
import { useEffect, useState } from "react";
import axios from "axios";

export default function LaunchEditor({
    block,
    onChange,
    onDelete
}: {
    block: LaunchFileConfig;
    onChange: (updated: LaunchFileConfig) => void;
    onDelete: () => void;
}) {
    const [packages, setPackages] = useState<string[]>([]);
    const [launchFiles, setLaunchFiles] = useState<string[]>([]);
    const [launchArgs, setLaunchArgs] = useState<Record<string, string>[]>([]);
    const [newArgName, setNewArgName] = useState("");
    const [newArgValue, setNewArgValue] = useState("");


    useEffect(() => {
        axios.get(API_BASE + "/api/ros/packages")
            .then(res => setPackages(res.data))
            .catch(console.error);
    }, []);

    useEffect(() => {
        if (!block.package) return;
        axios.get(API_BASE + "/api/ros/launch_files", { params: { package: block.package } })
            .then(res => setLaunchFiles(res.data))
            .catch(console.error);
    }, [block.package]);

    useEffect(() => {
        if (!block.package || !block.path) return;
        axios.get(API_BASE + "/api/ros/launch_args", { params: { package: block.package, file: block.path } })
            .then(res => setLaunchArgs(res.data))
            .catch(console.error);
    }, [block.package, block.path]);

    const updateField = (updates: Partial<LaunchFileConfig>) => {
        onChange({ ...block, ...updates });
    };

    const updateArg = (key: string, value: string) => {
        onChange({
            ...block,
            arguments: {
                ...(block.arguments || {}),
                [key]: value
            }
        });
    };

    const removeArg = (key: string) => {
        const newArgs = { ...(block.arguments || {}) };
        delete newArgs[key];

        onChange({
            ...block,
            arguments: newArgs
        });
    };

    return (
        <Box border={1} borderRadius={2} borderColor="grey.300" p={2}>
            <Stack spacing={2}>
                <Box display="flex" justifyContent="space-between" alignItems="center">
                    <Typography fontWeight={600}>Launch File</Typography>
                    <IconButton onClick={onDelete}><DeleteIcon /></IconButton>
                </Box>

                <Select
                    fullWidth
                    displayEmpty
                    value={block.package}
                    onChange={(e) => updateField({ package: e.target.value, path: "", arguments: {} })}
                >
                    <MenuItem value="">Select Package</MenuItem>
                    {packages.map((pkg) => (
                        <MenuItem key={pkg} value={pkg}>{pkg}</MenuItem>
                    ))}
                </Select>

                <Select
                    fullWidth
                    displayEmpty
                    value={block.path}
                    onChange={(e) => updateField({ path: e.target.value })}
                    disabled={!block.package}
                >
                    <MenuItem value="">Select Launch File</MenuItem>
                    {launchFiles.map((file) => (
                        <MenuItem key={file} value={file}>{file}</MenuItem>
                    ))}
                </Select>


                <Typography variant="subtitle2" gutterBottom>Set Launch Arguments</Typography>

                <FormControl fullWidth sx={{ mb: 1 }}>
                    <InputLabel>Select Argument</InputLabel>
                    <Select
                        value={newArgName}
                        onChange={(e) => setNewArgName(e.target.value)}
                        label="Select Argument"
                    >
                        {launchArgs
                            .filter((arg) => !(arg.name in block.arguments)) // don't show already-set args
                            .map((arg) => (
                                <MenuItem key={arg.name} value={arg.name}>
                                    {arg.name} — {arg.description}
                                </MenuItem>
                            ))}
                    </Select>
                </FormControl>

                <TextField
                    fullWidth
                    label="Value"
                    value={newArgValue}
                    onChange={(e) => setNewArgValue(e.target.value)}
                    sx={{ mb: 1 }}
                />

                <Button
                    variant="outlined"
                    disabled={!newArgName || !newArgValue}
                    onClick={() => {
                        updateArg(newArgName, newArgValue);
                        setNewArgName("");
                        setNewArgValue("");
                    }}
                >
                    Add Argument
                </Button>

                {/* Show current arguments */}
                {Object.entries(block.arguments || {}).map(([key, value]) => (
                    <Box key={key} display="flex" alignItems="center" mt={2} gap={2}>
                        <TextField label={key} value={value} fullWidth disabled />
                        <Button color="error" onClick={() => removeArg(key)}>Remove</Button>
                    </Box>
                ))}

            </Stack>
        </Box>
    );
}
