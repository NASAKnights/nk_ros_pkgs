import React from "react";
import ReactDOM from "react-dom/client";
import App from "./App";
import { ThemeProvider, CssBaseline } from "@mui/material";
import darkTheme from "./theme"; // 👈 import your custom theme

ReactDOM.createRoot(document.getElementById("root")!).render(
  <React.StrictMode>
    <ThemeProvider theme={darkTheme}>
      <CssBaseline /> {/* resets + enables dark background */}
      <App />
    </ThemeProvider>
  </React.StrictMode>
);
