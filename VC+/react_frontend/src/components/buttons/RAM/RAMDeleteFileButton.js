import DeleteSweepIcon from "@mui/icons-material/DeleteSweep";
import { Button } from "@mui/material";
import * as React from "react";
import PropTypes from "prop-types";

export const RAMDeleteFileButton = (props) => {
  const { deleteInDirectory } = React.useContext(props.context);
  return (
    <Button
      variant="contained"
      sx={{ m: 1 }}
      color={"secondary"}
      startIcon={<DeleteSweepIcon />}
      component="label"
      onClick={deleteInDirectory}
    >
      Delete file
    </Button>
  );
};

RAMDeleteFileButton.propTypes = {
  context: PropTypes.any,
};
