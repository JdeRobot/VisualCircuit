/**
 * Helper to create an URI to text data blob, so that it can be downloaded as file.
 * @param data Text data to be made into a blob
 * @param mimeType File type to be used.
 * @returns Data URI to the blob created from text
 */
export function textFile2DataURL(data: string, mimeType: string): string {
    const blob = new Blob([data], { type: mimeType });
    return URL.createObjectURL(blob);
}

/**
 * Generate an unique (pseudo-random) string of specified length
 * Courtesy of https://stackoverflow.com/a/1349426
 * @param length Length of the ID string required
 * @returns Unique string ID (Hopefully unique because its just random, and no guarantees)
 */
export function makeid(length: number) {
    var result = '';
    var characters = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789';
    var charactersLength = characters.length;
    for (var i = 0; i < length; i++) {
        result += characters.charAt(Math.floor(Math.random() *
            charactersLength));
    }
    return result;
}