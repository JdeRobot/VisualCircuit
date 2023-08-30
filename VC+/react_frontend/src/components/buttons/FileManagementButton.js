import * as React from 'react';
import '../../styles/styles.css';
import PropTypes from "prop-types";
import { Button } from "@mui/material";
import FolderIcon from "@mui/icons-material/Folder";
import StyleIcon from "@mui/icons-material/Style"
import AssistantIcon from "@mui/icons-material/Assistant"
import { RAMLoadFileButton } from ".//RAM/RAMLoadFileButton";
import { RAMCopyFileButton } from ".//RAM/RAMCopyFileButton";
import { RAMDeleteFileButton } from ".//RAM/RAMDeleteFileButton";
import { RAMCreateFileButton } from ".//RAM/RAMCreateFileButton";
import { SaveButton } from "./SaveButton";

const FileManagementButton = (props) => {
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
      sx={{ m: 1, "whiteSpace": "nowrap", "textAlign": "center"}}
      onClick={handleOpen}
      startIcon={<FolderIcon/>}
      >File Management</Button>
      {open ? (
        <ul className="menu" style = {{zIndex: 9999}}>
          <li className="menu-item">
            <RAMLoadFileButton context={props.context} />
          </li>
          <li className="menu-item">
            <RAMCopyFileButton context={props.context} />
          </li>
          <li className="menu-item">
            <RAMDeleteFileButton context={props.context} />
          </li>
          <li className="menu-item">
            <RAMCreateFileButton context={props.context} />
          </li>
          <li className="menu-item">
            <SaveButton context={props.context} />
          </li>
        </ul>
      ) : null}
    </div>
  );
};

export default FileManagementButton;