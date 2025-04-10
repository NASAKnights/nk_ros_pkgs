export interface NodeConfig {
    type: "node";
    name: string;
    namespace?: string;
    package: string;
    executable: string;
    parameters: Record<string, string>;
    remaps: Record<string, string>;
  }
  
  export interface LaunchFileConfig {
    type: "launch";
    package: string;
    path: string;
    arguments: Record<string, string>;
  }
  
  export type ConfigBlock = NodeConfig | LaunchFileConfig;
  
  export interface FullConfig {
    name: string;
    blocks: ConfigBlock[];
  }

  export const API_BASE = `http://${window.location.hostname}:8080`;