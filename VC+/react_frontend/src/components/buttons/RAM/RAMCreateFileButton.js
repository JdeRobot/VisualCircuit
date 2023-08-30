import AddIcon from "@mui/icons-material/Add";
import { Button, Box, TextField } from "@mui/material";
import { Modal } from "react-bootstrap";
import * as React from "react";
import { useState } from 'react';
import PropTypes from "prop-types";

export const RAMCreateFileButton = (props) => {
  const { deleteInDirectory, projectURL, changeFlagDirectoryView } = React.useContext(props.context);

  const [show, setShow] = React.useState(false);

  const [filename, setFilename] = React.useState("choose a filename");

  const handleClose = () => setShow(false);
  const handleShow = () => setShow(true);

  const handleSave = () => {
    var folder = "";
    if(filename.endsWith('.world')){
        folder = "world";
    }
    else{
        folder = "application";
    }
    fetch('http://127.0.0.1:8000/exercises/exercise/create_empty_file/' + projectURL + '/' + folder + '/' + filename)
    setShow(false);
    changeFlagDirectoryView(true);
  }

  return (
    <Box>
        <Button
          variant="contained"
          sx={{ m: 1 }}
          color={"secondary"}
          startIcon={<AddIcon />}
          component="label"
          onClick={handleShow}
        >
          Create file
        </Button>
        <Modal show={show} onHide={handleClose}>
          <Modal.Header closeButton>Create a new File</Modal.Header>
          <TextField
                sx={{ m: "6px" }}
                size={"small"}
                id="filename"
                label="Filename"
                color={"secondary"}
                value={filename}
                onChange={(e) => {
                  setFilename(e.target.value)
                }}
          />
          <Modal.Footer>
            <Button onClick={handleClose}>Close</Button>
            <Button onClick={handleSave}>Save</Button>
          </Modal.Footer>
        </Modal>
    </Box>
  );
};

RAMCreateFileButton.propTypes = {
  context: PropTypes.any,
};