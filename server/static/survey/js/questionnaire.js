// questionnaire.js: defined the class for the gameboard

// to add your own questions, define functions below

class Questionnaire {
    constructor(stage) {
        this.stage = stage;

        this.numElements = 0;  // the number of question elements until can move on, set by each question
        this.question = -1;

        this.results = [];

        this.questionDiv = document.getElementById("question");
        this.responseDiv = document.getElementById("response");

        this.previousQuestion = undefined;
        this.currentQuestion = undefined;
        this.nextQuestion = this.introduction;

        this.questionStack = [this.introduction];
    
        // when the next button is pressed, add this question to the stack
        document.getElementById("next-button").onclick = () => {
            this.reset();  // clear the questionnaire display
            this.questionStack.push(this.nextQuestion);  // add the next question to the stack
            this.nextQuestion();  // load the next question
        }

        // when the previous button is pressed, use the previous question from the stack
        document.getElementById("previous-button").onclick = () => {
            // don't run if at the first item
            if (this.questionStack.length == 1) {
                return;
            }

            this.reset();  // clear the questionnaire display
            this.currentQuestion = this.questionStack.pop();  // pop the current question off the stack
            this.previousQuestion = this.questionStack[this.questionStack.length - 1];  // the last item on the popped stack is the previous question
            this.previousQuestion();  // load the previous question
        }
    }

    // resets the questionnaire display
    reset() {
        this.questionDiv.innerHTML = "";
        this.responseDiv.innerHTML = "";
        this.numElements = 0;
    }

    // introduction
    introduction() {
        // set the questions
        this.questionDiv.innerHTML = "Welcome to our study!";
        
        // set the next question
        this.nextQuestion = this.ageSexGender;

        // set the buttons
        document.getElementById("previous-button").style.display = "none";
        document.getElementById("next-button").style.display = "flex";
    }

    // BACKGROUND

    // question 1: age, sex, gender
    ageSexGender() {
        // set the questions
        this.questionDiv.innerHTML = "Please answer the following:";

        createEntry("To what email should we send your compensation?", "email");
        createDropdown("What is your age?", ["18 - 25", "25 - 30", "30 - 35", "35 - 40", "40 - 45", "45+"]);
        createDropdown("What is your gender?", ["Female", "Male", "Other, or Unspecified"]);
        createDropdown("How familiar are you with robots?", ["Have never interacted with a robot", "Have interacted with a robot 1-2 times", "Have interacted with and programmed robots", "Have interacted with robots frequently"]);
        createSpace();
        createPixelCheckButton(1323, 919);
        
        // set the next question
        this.nextQuestion = this.ending;

        // set the buttons
        document.getElementById("previous-button").style.display = "flex";
        document.getElementById("next-button").style.display = "flex";
    }


    // end of questionnaire
    ending() {
        // set the questions
        this.questionDiv.innerHTML = "That's all! Please notify the researcher that you have completed.";

        // set the buttons
        document.getElementById("previous-button").style.display = "flex";
        document.getElementById("next-button").style.display = "none";
    }

}


