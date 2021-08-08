import { Button, Dialog, DialogActions, DialogContent, DialogContentText, TextField } from '@material-ui/core';
import React, { ChangeEvent, useState } from 'react';
import { create, InstanceProps } from 'react-modal-promise';
import { ProjectInfo } from '../../core/constants';

interface ProjectInfoDialogProps extends InstanceProps<ProjectInfo>, Partial<ProjectInfo> { };

/**
 * 
 * @param {
 *          isOpen: True if modal needs to be opened.
 *          onResolve: Will be called to indicate success / completion.
 *          onReject: Will be called to indicate failure.
 *          name: Name of project
 *          version: Version of block / package
 *          description: Description of project / package
 *          author: Author of project
 *          image: Icon for the project
 *        }
 */
const ProjectInfoDialog = ({ isOpen, onResolve, onReject,
    name, version, description, author, image }: ProjectInfoDialogProps) => {

    // Name of package. Use empty string if not defined
    const [nameInput, setName] = useState(name || '');
    // Version of package. Use empty string if not defined
    const [versionInput, setVersion] = useState(version || '');
    // Description of package. Use empty string if not defined
    const [descriptionInput, setDescription] = useState(description || '');
    // Author of package. Use empty string if not defined
    const [authorInput, setAuthor] = useState(author || '');
    // Icon of package. Use empty string if not defined
    const [imageInput, setImage] = useState(image || '');

    const fileReader = new FileReader();
    fileReader.onload = (event) => {
        if (event.target?.result) {
            setImage(event.target?.result.toString());
        }
    };

    /**
     * Callback for file uploading.
     * It reads the file blob as data URI
     * @param event File input event
     */
    const onFileUpload = (event: ChangeEvent<HTMLInputElement>) => {
        const file = event.target.files?.length ? event.target.files[0] : null;

        if (file) {
            fileReader.readAsDataURL(file);
        }
    }

    /**
     * Callback for 'Ok' button of the dialog
     */
    const handleSubmit = () => {
        // Send the filled data back
        onResolve({
            name: nameInput,
            version: versionInput,
            description: descriptionInput,
            author: authorInput,
            image: imageInput
        });
    }

    return (
        <Dialog
            open={isOpen}
            fullWidth={true}
            maxWidth='md'
            aria-labelledby="form-dialog-title">

            <DialogContent>
                <DialogContentText>
                    Name
                </DialogContentText>
                <TextField
                    autoFocus
                    margin="dense"
                    type="text"
                    variant='outlined'
                    value={nameInput}
                    onChange={(event) => setName(event.target.value)}
                    fullWidth
                />

                <DialogContentText>
                    Version
                </DialogContentText>
                <TextField
                    autoFocus
                    margin="dense"
                    type="text"
                    variant='outlined'
                    value={versionInput}
                    onChange={(event) => setVersion(event.target.value)}
                    fullWidth
                />

                <DialogContentText>
                    Description
                </DialogContentText>
                <TextField
                    autoFocus
                    margin="dense"
                    type="text"
                    variant='outlined'
                    value={descriptionInput}
                    onChange={(event) => setDescription(event.target.value)}
                    fullWidth
                />

                <DialogContentText>
                    Author
                </DialogContentText>
                <TextField
                    autoFocus
                    margin="dense"
                    type="text"
                    variant='outlined'
                    value={authorInput}
                    onChange={(event) => setAuthor(event.target.value)}
                    fullWidth
                />
                <DialogContentText>
                    Image
                </DialogContentText>
                <div style={{ display: 'flex', alignItems: 'center' }}>
                    <div style={{ flex: 1 }}>
                        <Button
                            variant="outlined"
                            component="label"
                        >
                            Upload File
                            <input
                                type="file"
                                accept='.svg'
                                onChange={onFileUpload}
                                hidden
                            />
                        </Button>
                    </div>
                    {imageInput &&
                        <img src={imageInput} style={{ width: '80px', height: '80px' }} alt='block icon' />}
                </div>
            </DialogContent>
            <DialogActions>
                <Button onClick={() => onReject()}>
                    Cancel
                </Button>
                <Button onClick={() => handleSubmit()}>
                    Ok
                </Button>
            </DialogActions>
        </Dialog>
    )
}

const createProjectInfoDialog = create(ProjectInfoDialog);

export default createProjectInfoDialog;