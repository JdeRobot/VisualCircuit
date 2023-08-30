import FileCopyIcon from "@mui/icons-material/FileCopy";
import { Button } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";

export const RAMCopyFileButton = (props) => {
  const { copyInDirectory } = React.useContext(props.context);
  return (
    <Button
      variant="contained"
      sx={{ m: 1 }}
      color={"secondary"}
      startIcon={<FileCopyIcon />}
      component="label"
      onClick={copyInDirectory}
    >
      Copy file
    </Button>
  );
};

RAMCopyFileButton.propTypes = {
  context: PropTypes.any,
};
