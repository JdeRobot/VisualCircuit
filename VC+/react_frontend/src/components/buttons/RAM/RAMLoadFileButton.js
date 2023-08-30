import UploadFileIcon from "@mui/icons-material/UploadFile";
import { Button } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";

export const RAMLoadFileButton = (props) => {
  const { loadInDirectory } = React.useContext(props.context);
  return (
    <Button
      variant="contained"
      sx={{ m: 1 }}
      color={"secondary"}
      startIcon={<UploadFileIcon />}
      component="label"
    >
      Load file
      <input hidden accept=".py, .js, .txt, .css, .json" type="file" onChange={loadInDirectory} />
    </Button>
  );
};

RAMLoadFileButton.propTypes = {
  context: PropTypes.any,
};
