{
    function formRefresh() {
        if (document.getElementById('step').value === '1') {
            document.getElementById('operation_data_form').hidden = false;
            document.getElementById('select_users').hidden = true;
            document.getElementById('select_drones').hidden = true;
            document.getElementById('select_devices').hidden = true;
            document.getElementById('select_baloras').hidden = true;
        } else if (document.getElementById('step').value === '2') {
            document.getElementById('operation_data_form').hidden = true;
            document.getElementById('select_users').hidden = false;
            document.getElementById('select_drones').hidden = true;
            document.getElementById('select_devices').hidden = true;
            document.getElementById('select_baloras').hidden = true;
        } else if (document.getElementById('step').value === '3') {
            document.getElementById('operation_data_form').hidden = true;
            document.getElementById('select_users').hidden = true;
            document.getElementById('select_drones').hidden = false;
            document.getElementById('select_devices').hidden = true;
            document.getElementById('select_baloras').hidden = true;
        } else if (document.getElementById('step').value === '4') {
            document.getElementById('operation_data_form').hidden = true;
            document.getElementById('select_users').hidden = true;
            document.getElementById('select_drones').hidden = true;
            document.getElementById('select_devices').hidden = false;
            document.getElementById('select_baloras').hidden = true;
        } else if (document.getElementById('step').value === '5') {
            document.getElementById('operation_data_form').hidden = true;
            document.getElementById('select_users').hidden = true;
            document.getElementById('select_drones').hidden = true;
            document.getElementById('select_devices').hidden = true;
            document.getElementById('select_baloras').hidden = false;
        }
        /**
         * Hide all form steps.
         */
        document.querySelectorAll('.form-step').forEach((formStepElement) => {
            formStepElement.classList.add('d-none');
        });
        /**
         * Mark all form steps as unfinished.
         */
        document.querySelectorAll('.form-stepper-list').forEach((formStepHeader) => {
            formStepHeader.classList.add('form-stepper-unfinished', 'text-muted');
            formStepHeader.classList.remove('form-stepper-active', 'form-stepper-completed');
        });

        /**
         * Select the form step circle (progress bar).
         */
        const formStepCircle = document.querySelector('li[step="' + document.getElementById('step').value + '"]');
        /**
         * Mark the current form step as active.
         */
        formStepCircle.classList.remove('form-stepper-unfinished', 'form-stepper-completed', 'text-muted');
        formStepCircle.classList.add('form-stepper-active');
        /**
         * Loop through each form step circles.
         * This loop will continue up to the current step number.
         * Example: If the current step is 3,
         * then the loop will perform operations for step 1 and 2.
         */
        for (let index = 1; index < document.getElementById('step').value; index++) {
            /**
             * Select the form step circle (progress bar).
             */
            const formStepCircle = document.querySelector('li[step="' + index + '"]');
            /**
             * Check if the element exist. If yes, then proceed.
             */
            if (formStepCircle) {
                /**
                 * Mark the form step as completed.
                 */
                formStepCircle.classList.remove('form-stepper-unfinished', 'form-stepper-active');
                formStepCircle.classList.add('form-stepper-completed');
            }
        }
        buttonRefresh();
    }
    function buttonRefresh() {
        if (document.getElementById('step').value === '1') {
            document.getElementById('previous').style.display = 'none';
            document.getElementById('next').style.display = '';
            document.getElementById('submit').style.display = 'none';
        } else if (document.getElementById('step').value === '5') {
            document.getElementById('previous').style.display = '';
            document.getElementById('next').style.display = 'none';
            document.getElementById('submit').style.display = '';
        } else {
            document.getElementById('previous').style.display = '';
            document.getElementById('next').style.display = '';
            document.getElementById('submit').style.display = 'none';
        }
    }
    function buttonNext() {
        if (document.getElementById('step').value !== 5) {
            document.getElementById('step').value = (parseInt(document.getElementById('step').value) + 1).toString();
        }
        formRefresh();
    }
    function buttonPrevious() {
        if (document.getElementById('step').value !== 1) {
            document.getElementById('step').value = (parseInt(document.getElementById('step').value) - 1).toString();
        }
        formRefresh();
    }
    function changeCustomStep(step) {
        document.getElementById('step').value = step;
        formRefresh();
    }

    Array.from(document.getElementsByClassName('form-click-button')).forEach((element) => {
        element.addEventListener('click', function (e) {
            if (!element.parentElement.parentElement.classList.contains('form-stepper-unfinished')) {
                changeCustomStep(element.parentElement.parentElement.getAttribute('step'));
            }
        });
    });

    formRefresh();
}
