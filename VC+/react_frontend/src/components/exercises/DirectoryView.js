import * as React from 'react';
import TreeView from '@mui/lab/TreeView';
import ExpandMoreIcon from '@mui/icons-material/ExpandMore';
import ChevronRightIcon from '@mui/icons-material/ChevronRight';
import TreeItem from '@mui/lab/TreeItem';
import { Box, TextField, Button } from "@mui/material";
import { Modal } from "react-bootstrap";

export default function DirectoryView(props) {

  const { userCodeChanged, codeChanged, editorCode, editorCodeChange, displayDirectory, projectURL, changeSelectedNodeId,
  handleWorldChange, loadCode, changeEntryPoint, changeAppSelected, changeWorldSelected, flagDirectoryView, changeFlagDirectoryView } = React.useContext(props.context);

  const csrftoken = document.querySelector('[name=csrfmiddlewaretoken]').value;

  const [show, setShow] = React.useState(false);

  const [filename, setFilename] = React.useState("choose a filename");

  const handleClose = () => setShow(false);
  const handleShow = () => setShow(true);

  const [renamePath, setRenamePath] = React.useState("");

  const handleRename = () => {
    var string = renamePath.split('/');
    fetch('http://127.0.0.1:8000/exercises/exercise/rename_file/' + projectURL + '/' + string[1] + '/',{
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
          'X-CSRFToken': csrftoken
        },
        body: JSON.stringify({ newName: filename, oldName: string[2]})
    });
    changeFlagDirectoryView(true);
    setShow(false);
  }

  React.useEffect(() => {
    const interval = setInterval(() => {
    if(codeChanged == true){
        userCodeChanged(false);
        fetch('http://127.0.0.1:8000/exercises/exercise/save_data/', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
              'X-CSRFToken': csrftoken
            },
            body: JSON.stringify({ data: editorCode })            
        })
      }
    }, 1000);

    return () => {
      clearInterval(interval);
    };
  }, [codeChanged]);

  function FolderTreeItem({ item }) {

      var guardado = false

      const saveChild = async (path) => {
        const response = await fetch('http://127.0.0.1:8000/exercises/exercise/save_file_content/' + filePath + '/' + path.slice(1) + '/', {
            method: 'POST',
            headers: {
              'Content-Type': 'application/json',
              'X-CSRFToken': csrftoken
            },
            body: JSON.stringify({ data: editorCode })
        })
        const result = await response.text()
        if (result == "Archivo no guardable") {
            console.log("Has intentado abrir un archivo que no es de texto")
        } else {
            if (result == "No se ha podido guardar") {
              console.log("El código esta guardado en un editor no asociado a ningún archivo")
              console.log(editorCode)
            }
            guardado = true
        }
      }

      const TreeChild = async (event, path) => {
        event.stopPropagation()

        await saveChild(path)

        if(guardado == true){
          guardado = false
          fetch('http://127.0.0.1:8000/exercises/exercise/get_file_content/' + filePath + '/' + path.slice(1) +'/')
              .then(response => response.text())
              .then(function(result){
                  editorCodeChange(result)
              })
              .catch(error => {
                  console.error(error);
              });
        }
      };

      const handleTreeItemClick = (path) => (event) => {
        setRenamePath(path);
        event.preventDefault();
        var string = path.split("/");
        if(event.type === 'click'){
          if(event.detail == 1){
		    changeSelectedNodeId(path);
            TreeChild(event, path);
		  }else if(event.detail == 2){
			if(path.endsWith('.world')){
			    var string2 = string[2].split(".");
			    changeWorldSelected(string2[0]);
                fetch(
                  'http://127.0.0.1:8000/exercises/exercise/get_file_content/' +
                    filePath +
                    path
                )
                  .then((response) => response.text())
                  .then((result) => {
                    handleWorldChange(result);
                    fetch(
                    'http://127.0.0.1:8000/exercises/exercise/save_metadata_file/', {
                    method: 'POST',
                    headers: {
                    'Content-Type': 'application/json',
                    'X-CSRFToken': csrftoken
                    },
                    body: JSON.stringify({ url: projectURL+"/metadata/", data: result, name: string2[0] })
                    })
                    .then((response) => response.text())
                    .then((result) => {
                        console.log(result);
                    })
                  })
                  .catch((error) => {
                    console.error(error);
                  });
			}else if(path.endsWith('.py')){
			    changeEntryPoint(string[2]);
                changeAppSelected(string[2]);
                loadCode();
                console.log("Ejecutando el archivo seleccionado");
			}else{
			    console.log("Este formato de archivo no es ejecutable");
			}
		  }
        }else if(event.type === 'contextmenu'){
          var string = path.split("/");
          setFilename(string[2]);
          setShow(true)
        }
      }

      return (
        // <TreeItem
        //   nodeId={item.path}
        //   label={item.name}
        //   defaultcollapseicon={<ExpandMoreIcon />}
        //   defaultexpandicon={<ChevronRightIcon />}
        // >
        //   {item.children.map(child => {
        //     if (child.type === 'directory') {
        //       return <FolderTreeItem key={child.path} item={child} />;
        //     } else {
        //       return (
        //         // <TreeItem
        //         //   key={child.path}
        //         //   nodeId={child.path}
        //         //   label={child.name}
        //         //   href={child.path}
        //         //   onClick={handleTreeItemClick(child.path)}
        //         //   onContextMenu={handleTreeItemClick(child.path)}
        //         // />
        //         <div>hii</div> 
        //       );
        //     }
        //   })}
        // </TreeItem>
        <></>
      );
  }

  function filesToTree(files) {
    const tree = [];
  
    files.forEach(file => {
      const pathSegments = window.navigator.platform.startsWith('Linux') ? file.split('/') : file.split('\\');
      let currentLevel = tree;
  
      pathSegments.forEach((segment, index) => {
        const existingItem = currentLevel.find(item => item.name === segment);
  
        if (existingItem) {
          currentLevel = existingItem.children;
        } else {
          const newItem = {
            name: segment,
            path: pathSegments.slice(0, index + 1).join('/'),
            type: index === pathSegments.length - 1 ? 'file' : 'directory',
            children: [],
          };
          currentLevel.push(newItem);
          currentLevel = newItem.children;
        }
      });
    });
  
    const sortTree = (items) => {
      items.sort((a, b) => {
        if (a.type === 'directory' && b.type === 'file') return -1;
        if (a.type === 'file' && b.type === 'directory') return 1;
        return 0;
      });
  
      items.forEach(item => {
        if (item.type === 'directory') {
          sortTree(item.children);
        }
      });
    };
  
    sortTree(tree);
    return tree;
  }


  const filePath = projectURL;
  const [files, setFiles] = React.useState([]);

  React.useEffect(() => {
    if(filePath != ""){
        fetch('http://127.0.0.1:8000/exercises/exercise/get_folder_content/' + filePath)
          .then(response => response.json())
          .then(data => {
            setFiles(data.files);
          })
          .catch(error => {
            console.log("Loading...");
          });
    }
    changeFlagDirectoryView(false);
  }, [codeChanged, projectURL, flagDirectoryView]);

  const fileTree = filesToTree(files);

  return (
    <Box
      sx={{
        m: 1,
        width: "100%",
        flexDirection: "column",
        display: displayDirectory,
        justifyContent: 'start',
        backgroundColor: '#6495ED',
        color:'black',
      }}
    >
      <TreeView
        defaultCollapseIcon={<ExpandMoreIcon />}
        defaultExpandIcon={<ChevronRightIcon />}
      >
        {fileTree.map(item => (
          <FolderTreeItem key={item.path} item={item} />
        ))}
      </TreeView>
      <Modal show={show} onHide={handleClose}>
          <Modal.Header closeButton>Rename file</Modal.Header>
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
            <Button onClick={handleRename}>Save</Button>
          </Modal.Footer>
        </Modal>
    </Box>
  );
}