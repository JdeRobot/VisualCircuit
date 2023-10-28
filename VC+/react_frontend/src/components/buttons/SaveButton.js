import * as React from "react";
import SaveIcon from "@mui/icons-material/Save";
import { Button, TextField, Box } from "@mui/material";
import { saveCode } from "../../helpers/utils";
import PropTypes from "prop-types";
import { Modal } from "react-bootstrap";

export const SaveButton = (props) => {
  const { editorCode } = React.useContext(props.context);

  const [filename, setFilename] = React.useState("Save file as");

  const [show, setShow] = React.useState(false);

  const handleClose = () => setShow(false);
  const handleShow = () => setShow(true);

  const handleSavePc = () => {
    saveCode(filename, editorCode);
    handleClose;
  }

  return (
    <Box>
        <Button
          id={"save"}
          variant="contained"
          color={"secondary"}
          startIcon={<SaveIcon />}
          sx={{ m: 1 }}
          onClick={handleShow}
        >
          Save File
        </Button>
        <Modal show={show} onHide={handleClose}>
          <Modal.Header closeButton>Save file on computer</Modal.Header>
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
            <Button onClick={handleSavePc}>Save</Button>
          </Modal.Footer>
        </Modal>
    </Box>
  );
};

SaveButton.propTypes = {
  context: PropTypes.any,
};
