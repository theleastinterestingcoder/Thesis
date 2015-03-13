#! /usr/bin/emacs  --script
;;-*- mode: emacs-lisp;-*-

(defun process (string)
  "just reverse the string"
  (concat (nreverse (string-to-list string))))

(condition-case nil
    (let (line)
      ;; commented out b/c not relevant for `cat`, but potentially useful
      ;; (princ "argv is ")
      ;; (princ argv)
      ;; (princ "\n")
      ;; (princ "command-line-args is" )
      ;; (princ command-line-args)
      ;; (princ "\n")

      (while (setq line (read-from-minibuffer ""))
        (princ (process line))
        (princ "\n")))
  (error nil))

