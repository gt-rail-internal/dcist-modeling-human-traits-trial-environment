// elements.js contains functions for generating form response elements

function createDropdown(question, items) {
    // create the element tag in case we need to reference it
    let tag = question.replace(/\W/g, '');
    
    // create the overall dropdown div
    let overallDiv = document.createElement("div");
    overallDiv.id = "dropdown-" + tag + "-div"

    // create the question label
    let label = document.createElement("label");
    label.for = "dropdown-" + tag;
    label.innerHTML = question;
    label.classList = ["dropdown"];

    // add the label to the overall div
    overallDiv.appendChild(label);
    
    // create the dropdown selector
    let select = document.createElement("select");
    select.name = "dropdown-" + tag;
    select.id = "dropdown-" + tag;
    select.classList = ["dropdown"];

    // create the default select option
    let option = document.createElement("option");
    option.value = -1;
    option.innerHTML = "Select One";
    option.selected = true;
    option.disabled = true;
    select.appendChild(option);

    // create the other options
    for (i in items) {
        option = document.createElement("option");
        option.value = i;
        option.innerHTML = items[i];
        option.classList = ["dropdown"];
        select.appendChild(option);
    }

    // set the onchange to log and check elements
    select.onchange = () => {
        log({"tag": tag, "type": "dropdown", "selection": select.selectedIndex});

        if (!overallDiv.dataset.used) {
            overallDiv.dataset.used = true;
            checkElements();
        }
    };

    // add the selector to the overall div
    overallDiv.appendChild(select);

    // add the overall div to the response element
    document.getElementById("response").appendChild(overallDiv);

    questionnaire.numElements += 1;
    return "dropdown-" + tag;
}


function createEntry(question, tag) {    
    // create the overall dropdown div
    let overallDiv = document.createElement("div");
    overallDiv.id = "entrylabel-" + tag + "-div"

    // create the question label
    let label = document.createElement("label");
    label.for = "entrylabel-" + tag;
    label.innerHTML = question;
    label.classList = ["dropdown"];

    // add the label to the overall div
    overallDiv.appendChild(label);
    
    // create the input field
    let input = document.createElement("input");
    input.name = "entry-" + tag;
    input.id = "entry-" + tag;
    input.classList = ["entry"];

    // set the onchange to log and check elements
    input.onchange = () => {
        log({"tag": tag, "type": "entry", "input": input.value});
    };

    // add the input to the overall div
    overallDiv.appendChild(input);

    // add the overall div to the response element
    document.getElementById("response").appendChild(overallDiv);

    return "entry-" + tag;
}


function createTableDropdown(question, items) {
    // create the element tag in case we need to reference it
    let tag = question.replace(/\W/g, '');
    
    // create the overall dropdown div
    let overallDiv = document.createElement("table");
    overallDiv.id = "dropdown-" + tag + "-div"
    overallDiv.dataset.used = false;
    let row = overallDiv.insertRow();

    // create the question label
    let label = document.createElement("label");
    label.for = "dropdown-" + tag;
    label.innerHTML = question;
    label.classList = ["dropdown"];

    // add the label to the overall div
    row.insertCell(0).appendChild(label);
    
    // create the dropdown selector
    let select = document.createElement("select");
    select.name = "dropdown-" + tag;
    select.id = "dropdown-" + tag;
    select.classList = ["dropdown"];

    // create the default select option
    let option = document.createElement("option");
    option.value = -1;
    option.innerHTML = "Select One";
    option.selected = true;
    option.disabled = true;
    select.appendChild(option);

    // create the other options
    for (i in items) {
        option = document.createElement("option");
        option.value = i;
        option.innerHTML = items[i];
        option.classList = ["dropdown"];
        select.appendChild(option);
    }

    // set the onchange to log and check elements
    select.onchange = () => {
        log({"tag": tag, "type": "dropdown", "selection": select.selectedIndex});
        if (!overallDiv.dataset.used) {
            overallDiv.dataset.used = true;
            checkElements();
        }
    };

    // add the selector to the overall div
    row.insertCell(1).appendChild(select);

    // add the overall div to the response element
    document.getElementById("response").appendChild(overallDiv);

    console.log(">>>", overallDiv);
    questionnaire.numElements += 1;
    return "dropdown-" + tag;
}


function createHorizontalSelector(question, items) {
    // create the element tag in case we need to reference it
    let tag = question.replace(/\W/g, '');
    
    // create the overall horizontal selector div
    let overallDiv = document.createElement("div");
    overallDiv.id = "horizontal-" + tag + "-div";
    overallDiv.dataset.used = false;

    // create the question label (shown above selector)
    let label = document.createElement("div");
    label.innerHTML = question;

    label_sub = document.createElement("div");
    label_sub.innerHTML = "Select the level below that most accurately applies to you:";
    label_sub.classList = ["horizontal-label"];

    // add the question to the overall div
    overallDiv.appendChild(label);
    overallDiv.appendChild(label_sub)

    // create the selector options' container
    optionContainer = document.createElement("div");
    optionContainer.classList = ["horizontal-container shadowed"]
    let options = [];

    // sets the selected class to the selected option
    function selectOption(i) {
        for (o in options) {
            if (o == i) {
                options[o].classList = ["horizontal-item selected"];
            }
            else {
                options[o].classList = ["horizontal-item"];
            }
        }
        // log the selection
        log({"tag": tag, "type": "horizontal", "selection": i});

        if (overallDiv.dataset.used == "false") {
            overallDiv.dataset.used = true;
            checkElements();
        }

    }

    // create the options for the container
    for (i in items) {
        let option = document.createElement("div");
        option.innerHTML = items[i];
        option.classList = ["horizontal-item"];
        option.dataset.index = i;
        option.onclick = () => { selectOption(option.dataset.index); };

        optionContainer.appendChild(option);
        options.push(option);
        
    }

    // add the options container to the overall div
    overallDiv.appendChild(optionContainer);

    // add the overall div to the response element
    document.getElementById("response").appendChild(overallDiv);

    console.log(">>>", overallDiv);
    questionnaire.numElements += 1;
    return "horizontal-" + tag;
}


function createHorizontalImageboard(tag, urls) {
    // create the element tag in case we need to reference it
    tag = tag.replace(/\W/g, '');
    
    // create the overall horizontal selector div
    let overallDiv = document.createElement("div");
    overallDiv.id = "horizontalimgs-" + tag + "-div"

    // create the selector options' container
    images = document.createElement("div");
    images.classList = ["horizontal-container"]

    // create the options for the container
    for (i in urls) {
        let img = document.createElement("img");
        img.src = urls[i];
        img.classList = ["horizontal-image"];

        images.appendChild(img);
    }

    // add the options container to the overall div
    overallDiv.appendChild(images);

    // add the overall div to the response element
    document.getElementById("response").appendChild(overallDiv);

    return "horizontalimgs-" + tag;
}

function createPixelCheckButton(width, height) {
    let overallDiv = document.createElement("div");
    overallDiv.id = "pixelcheck-div";
    overallDiv.dataset.used = false;
    overallDiv.classList = ["center-div"];

    let button = document.createElement("div");
    button.classList = ["horizontal-item"]
    button.style.width = "50%";
    button.style.textAlign = "center";
    button.style.display = "block";

    let targetWidth = 1323;
    let targetHeight = 919;

    button.innerHTML = "";
    window.setInterval(() => {
        if (window.innerWidth < targetWidth) {
            button.innerHTML = "Browser is too narrow! Widen by at least " + Math.floor((targetWidth / window.innerWidth - 1) * 100) + "%";
            button.style.backgroundColor = "lightcoral";
        }
        else if (window.innerHeight < targetHeight) {
            button.innerHTML = "Browser is too short! Heighten by at least " + Math.floor((targetHeight / window.innerHeight - 1) * 100) + "%";
            button.style.backgroundColor = "lightcoral";
        }
        else {
            button.innerHTML = "Your browser dimensions fit!"
            button.style.backgroundColor = "lightgreen";
        }
    }, 250);        

    overallDiv.append(button);

    // add the overall div to the response element
    document.getElementById("response").appendChild(overallDiv);
    return "pixelcheck";
}

function createSpace() {
    let overallDiv = document.createElement("div");
    overallDiv.id = "space-div";
    overallDiv.dataset.used = false;

    overallDiv.innerHTML = "<br>";

    // add the overall div to the response element
    document.getElementById("response").appendChild(overallDiv);
    return "space";
}


function log(data) {
    data["worker-id"] = data.tag == "email" ? "email" : workerId;
    console.log("LOG RAW", data);
    return fetch("/logging", {method: "POST", body: JSON.stringify(data)});
}


function checkElements() {
    questionnaire.numElements -= 1;
    if (questionnaire.numElements == 0) {
        document.getElementById("next-button").style.display = "flex";
    }
}