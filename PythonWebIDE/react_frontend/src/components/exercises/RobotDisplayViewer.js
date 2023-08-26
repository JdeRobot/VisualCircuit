import * as React from "react";
import PropTypes from "prop-types";

function RobotDisplayViewer() {
  return (
    <>
      <iframe
        className ="responsive-iframe"
        id={"iframe"}
        style={{
          width: "100%",
          height: "400px",
        }}
        src={"http://127.0.0.1:2303/vnc.html?resize=remote&autoconnect=true"}
      />
    </>
  );
}
RobotDisplayViewer.propTypes = {
  context: PropTypes.any,
};

export default RobotDisplayViewer;
