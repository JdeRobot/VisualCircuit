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

  return (
    <div>
      <Button
      id={"File_Management_Button"}
      size={"medium"}
      variant="contained"
      color={"secondary"}
      component="span"
      sx={{ m: 1 }}
      onClick={handleOpen}
      startIcon={<PageviewIcon/>}
      >Views </Button>
      {open ? (
        <ul className="menu" style = {{zIndex: 9999}}>
          <li className="menu-item">
            <CameraButton context={props.context} />
          </li>
          <li className="menu-item">
            <GazeboButton context={props.context} />
          </li>
          <li className="menu-item">
            <ConsoleButton context={props.context} />
          </li>
        </ul>
      ) : null}
    </div>
  );
};

export default ViewsLayoutButton;