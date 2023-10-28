import React, { useContext, useEffect, useState } from "react";
import PropTypes from "prop-types";
import PlayArrowIcon from "@mui/icons-material/PlayArrow";
import LoadingButton from "@mui/lab/LoadingButton";

const RAMPlay = (props) => {
  const { editorCode, setLinterMessage, loadCode, loading } = useContext(props.context);
  const [disabled, setDisabled] = useState(true);

  const csrftoken = document.querySelector('[name=csrfmiddlewaretoken]').value;

  useEffect(() => {
    const callback = (message) => {
      if (
        message.data.state === "ready" ||
        message.data.state === "paused" //||
        //message.data.state === "running"
      ) {
        setDisabled(false);
      } else {
        setDisabled(true);
      }
    };

    window.RoboticsExerciseComponents.commsManager.subscribe(
      [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
      callback
    );

    return () => {
      window.RoboticsExerciseComponents.commsManager.unsubscribe(
        [window.RoboticsExerciseComponents.commsManager.events.STATE_CHANGED],
        callback
      );
    };
  }, []);

  return (
    <LoadingButton
      disabled={disabled}
      id={"loadIntoRobot"}
      loading={loading}
      color={"secondary"}
      onClick={() => {
        loadCode();
      }}
      startIcon={<PlayArrowIcon />}
      sx={{ m: 0.5 }}
      variant={"outlined"}
      loadingPosition="start"
    >
      play
    </LoadingButton>
  );
};
RAMPlay.propTypes = {
  context: PropTypes.any,
};

export default RAMPlay;
