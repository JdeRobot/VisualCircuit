import * as React from 'react';
import CircleIcon from '@mui/icons-material/Circle';
import FileDownloadDoneIcon from '@mui/icons-material/FileDownloadDone';

const SaveFeedbackIcon = (props) => {

  const { codeChanged } = React.useContext(props.context);

  return (
    <>
      {codeChanged ? <CircleIcon/> : ""}
    </>
  );
};

export default SaveFeedbackIcon;
