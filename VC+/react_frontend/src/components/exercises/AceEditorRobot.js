import React from 'react';
import { Box } from '@mui/material';
import AceEditor from 'react-ace';
import 'ace-builds/src-noconflict/mode-python';
import 'ace-builds/src-noconflict/theme-dracula';
import PropTypes from 'prop-types';
class AceEditorRobot extends React.Component {
  constructor(props) {
    super(props);
    this.editorRef = React.createRef();
  }
  handleFileChange = async (event) => {
    const file = event.target.files[0];
    if (file && file.type === 'application/zip') {
      console.log('Zip file selected:', file.name);
      const base64Data = await this.convertFileToBase64(file);
      if (base64Data) {
        var entryPoint="/main.py";
        window.RoboticsExerciseComponents.commsManager.send("load", {
          zip: base64Data,
          entrypoint: entryPoint,
        })
      } else {
        console.log('Error converting zip file to base64.');
      }
    } else {
      console.log('Please select a valid zip file.');
    }
  };
  convertFileToBase64 = (file) => {
    return new Promise((resolve, reject) => {
      const reader = new FileReader();
      reader.onload = () => {
        const base64Data = reader.result.split(',')[1];
        resolve(base64Data);
      };
      reader.onerror = (error) => {
        reject(error);
      };
      reader.readAsDataURL(file);
    });
  };
  render() {
    const { editorCode, editorCodeChange, displayEditor } = this.props.context;
    return (
      <input
        type="file"
        accept=".zip"
        onChange={this.handleFileChange}
      />
    );
  }
}
AceEditorRobot.propTypes = {
  context: PropTypes.any,
};
export default AceEditorRobot;