import React, { useEffect, useState } from "react";

import MenuItem from "@mui/material/MenuItem";
import AppSettingsAltIcon from "@mui/icons-material/AppSettingsAlt";

import { FormControl, InputLabel, Select } from "@mui/material";

export default function ApplicationSelector(props) {

  const { projectURL, changeEntryPoint, appSelected, changeAppSelected } = React.useContext(props.context);

  const csrftoken = document.querySelector('[name=csrfmiddlewaretoken]').value;

  const[isOpen, setIsOpen] = useState(false)

  const [disabled, setDisabled] = useState(true);

  useEffect(() => {
    const callback = (message) => {
      if (message.data.state === "ready") {
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

  function MenuItems({ item, setIsOpen }) {
  const handleClick = () => {
    changeAppSelected(item.name);
    changeEntryPoint(item.name);
    setIsOpen(false); // close the menu
    console.log("Se ha seleccionado " + item.name + " como el entry point");
  };

  return <MenuItem onClick={handleClick} value={item.name}>{item.name}</MenuItem>;
}


  function MapItems(files){
    const items = []
    files.forEach(file => {
      const pathSegments = window.navigator.platform.startsWith('Linux') ? file.split('/') : file.split('\\');file.split('\\');
      const newItem = {
        name: pathSegments[1],
        path: pathSegments.slice(0).join('/'),
      };
      items.push(newItem);
      });
    return items;
  }

  const filePath = projectURL + "/application/"
  const [files, setFiles] = React.useState([]);

  React.useEffect(() => {
    fetch('http://127.0.0.1:8000/exercises/exercise/get_folder_content/' + filePath)
      .then(response => response.json())
      .then(data => {
        setFiles(data.files);
      })
      .catch(error => {
        console.error(error);
      });
  }, [isOpen]);

  const menuItems = MapItems(files);
  return (
      <>
        <FormControl>
          <InputLabel id={"application-selector-label"}>
            Application
            < AppSettingsAltIcon />
          </InputLabel>
          <Select
            disabled={disabled}
            defaultValue={"None"}
            labelId="application-selector-label"
            id={"application-selector"}
            label={"Application"}
            MenuProps={{
              onClose: () => setIsOpen(false),
            }}
            open={isOpen}
            onOpen={() => setIsOpen(true)}
          >
            <MenuItem selected disabled value="None"><em>{appSelected}</em></MenuItem>
            {menuItems.map((item) => (
              <MenuItems key={item.name} item={item} setIsOpen={setIsOpen} />
            ))}
          </Select>
        </FormControl>
      </>
  );
}
