import * as React from 'react';
import '../../styles/styles.css';
import PropTypes from "prop-types";
import { Button } from "@mui/material";
import PageviewIcon from "@mui/icons-material/Pageview";
import StyleIcon from "@mui/icons-material/Style"
import AssistantIcon from "@mui/icons-material/Assistant"
import CameraButton from "./RAMVisualizatorButton";
import GazeboButton from "./GazeboButton";
import ConsoleButton from "./ConsoleButton";

const ViewsLayoutButton = (props) => {
  const [open, setOpen] = React.useState(false);

  const handleOpen = () => {
    setOpen(!open);
  };

  return null;
};

export default ViewsLayoutButton;