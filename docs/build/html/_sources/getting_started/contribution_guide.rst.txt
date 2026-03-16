.. _contribution_guide:

Contribution Guide
====================

Contributing Code
###################

If you're adding new code features, please consider following the workflow below:

1. Write your code
    After setting up the project you can get to writing new code. When working on a new feature to add to the code, please create a new local branch off of ``dev`` dwith a descriptive name.
    When you have made progress on your code, push your local branch to the github repository.

2. Write Tests
    Untested code is broken code.

3. Write documentation
    Please write detailed documentation for any new classes, functions, etc. you create.

4. Submit a pull request
    After completing, testing, and documenting your code, submit a pull request with your finalized changes.

    Ensure your code passes all CI checks before submitting your pull request. You can check the status of your commits by checking the Actions tab in the Github web browser.

    After resolving all issues, request review from the github administrator, as this will be required for merging your code into the main code base.

Writing Documentation
#######################

Another way to contribute is to write documentation for existing code. Documentation for this project is located in the ``doc`` directory and is written with reStructuredText.
Sphinx is then used to generate these pages from the reStructuredText files.
The following resources can help familiarize you with writing documentation with reStructuredText:

- https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html
- https://docutils.sourceforge.io/docs/user/rst/quickref.html

Previewing Documentation
^^^^^^^^^^^^^^^^^^^^^^^^^^
This documentation is intended to serve as a guide to the S26-14 Framatome Quadrature Inclinometer Project.
Any changes made to the documentation and code can be previewed on your local machine by running:

::

    make html

inside the ``doc`` folder. If you made any changes to the Python code docstrings, make sure to re-install the Python code to see the documentation changes reflected.

